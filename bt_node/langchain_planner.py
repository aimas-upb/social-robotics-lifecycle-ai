from langchain_openai import ChatOpenAI
from langchain.schema import HumanMessage, SystemMessage
from langchain.prompts import PromptTemplate
from langchain_core.runnables import RunnablePassthrough, RunnableLambda, RunnableMap
from langchain_core.runnables.graph import CurveStyle, MermaidDrawMethod
from bt_node.config import Config
import os
from bt_node.templates import planning_template, brainstorm_system_template, brainstorm_human_template, plan_to_json_template, \
actions, rooms, people, examples, rules
from typing import Literal, TypedDict, Annotated, List, Union
from langchain_core.messages import BaseMessage, ToolMessage, AIMessage
from langchain.tools import BaseTool, StructuredTool, Tool, tool
from langchain_tavily import TavilySearch
from langchain import hub
from langgraph.graph import StateGraph, START, END
from langgraph.graph.message import add_messages
from langgraph.checkpoint.memory import MemorySaver

os.environ["LANGSMITH_TRACING"]="true"
os.environ["LANGSMITH_ENDPOINT"]="https://api.smith.langchain.com"
os.environ["LANGSMITH_API_KEY"]="YOUR-KEY-HERE"
os.environ["LANGSMITH_PROJECT"]="YOUR-PROJECT-NAME-HERE"
TAVILY_API_KEY="YOUR-KEY-HERE"
os.environ["OPENAI_API_KEY"] = "YOUR-KEY-HERE"

llm = ChatOpenAI(
    # model="gpt-4o"
    model="gpt-4.1",
    # model="o3-mini", #slow and unreliable
    # temperature=0.0,
    streaming=True)

class AgentState(TypedDict):
    messages: Annotated[list, add_messages]
    context: str
    feedback: bool


# === Tools ===

@tool("request_details", return_direct=True)
def request_details(task:str):
    """If you feel like the given goal is too different from any other you've seen before or just need a more detailed request is necessary for you to 
output a description, you may call this and recieve clarifications"""    
    
    details = input("Please provide clarifications: ")
    return {"messages": [HumanMessage(content="As a clarification, " + details)]}

search_tool = TavilySearch(
    max_results=1,
    topic="general",
    tavily_api_key=TAVILY_API_KEY
)

tools = [request_details, search_tool]
tools_by_name = {tool.name : tool for tool in tools}
llm_with_tools = llm.bind_tools(tools)


# === LLM Nodes ===

def invoke_brainstorm_agent(state):
    """Run the LLM to get the plan in natural language based on the input"""

    config = {"configurable": {"thread_id": "1"}}
    return {
        "messages": [
            llm_with_tools.invoke([
                SystemMessage(
                    content=brainstorm_system_template
                )
            ]
            + state["messages"],
            config=config
            )
        ]
    }

def invoke_planning_agent(state):
    """Run the LLM to generate the plan in a pseudocode-like format"""


    planning_prompt = PromptTemplate(input_variables=["situation", "actions", "rooms", "context", "people", "rules", "examples"], 
        template=planning_template)

    formatted_prompt = planning_prompt.format(
        situation=state["messages"][-1],
        actions=actions,
        rooms=rooms,
        context=state["context"],
        people=people,
        rules=rules,
        examples=examples
    )

    return {
        "messages": [
            llm.invoke([
                SystemMessage(content=formatted_prompt)                
            ]
            )
        ]
    }

def invoke_converter_agent(state):
    """Run the LLM to convert pseudocode-like plan to a JSON format"""

    plan_to_json_prompt = PromptTemplate(template=plan_to_json_template, input_variables=["plan"])

    formatted_prompt = plan_to_json_prompt.format(
        plan=state["messages"][-1]
    )

    return {
        "messages": [
            llm.invoke([
                SystemMessage(content=formatted_prompt)
            ])
        ]
    }

def get_task_feedback(state):
    """Gets feedback from user regarding the latest task completed"""
    feedback = input("How did I do?: ")
    return {
        "messages": [
            AIMessage(content="How did I do?"),
            HumanMessage(content=feedback)
        ]
    }

def tool_node(state):
    """Performs the tool call(s) dynamically based on tool name and args"""
    result = []
    last_message = state["messages"][-1]
    
    if hasattr(last_message, "tool_calls"):
        for tool_call in last_message.tool_calls:
            tool_name = tool_call["name"]
            tool_args = tool_call["args"]
            tool = tools_by_name[tool_name]
            
            observation = tool.invoke(tool_args)
            
            content = observation if isinstance(observation, str) else str(observation)
            
            result.append(
                ToolMessage(content=content, tool_call_id=tool_call["id"])
            )
    
    return {"messages": result}


def should_continue(state) -> Literal["tools", "plan"]:
    last_message = state["messages"][-1]

    if hasattr(last_message, "tool_calls") and last_message.tool_calls:
        return "tools"

    return "plan"

def check_feedback(state):
    return state["feedback"]


# === Graph Definition ===

graph_builder = StateGraph(AgentState)

graph_builder.add_node("brainstorm", RunnableLambda(invoke_brainstorm_agent))
graph_builder.add_node("tools", RunnableLambda(tool_node))
graph_builder.add_node("plan", RunnableLambda(invoke_planning_agent))
graph_builder.add_node("convert", RunnableLambda(invoke_converter_agent))
graph_builder.add_node("get_feedback", RunnableLambda(get_task_feedback))

graph_builder.add_conditional_edges(START, check_feedback,
                                    {
                                        False: "brainstorm",
                                        True: "get_feedback"
                                    })
graph_builder.add_edge("tools", "brainstorm")
graph_builder.add_conditional_edges("brainstorm",
                                    should_continue,
                                    {
                                        "tools": "tools",
                                        "plan": "plan"
                                    })
graph_builder.add_edge("plan", "convert")
graph_builder.add_edge("convert", END)
graph_builder.add_edge("get_feedback", END)

memory = MemorySaver()

graph = graph_builder.compile(checkpointer=memory)
# Save Mermaid syntax to a file
# mermaid_code = graph.get_graph().draw_mermaid()
# with open("graph_diagram.mmd", "w") as f:
#     f.write("```mermaid\n" + mermaid_code + "\n```")


# === Planner ===

def get_person_context(name, room):
    if not room:
        return f"You don't know where {name} was"

    if room == Config.current_room:
        return f"{name} is in {room.name} with you"
    
    return f"{name} was in {room.name}"

def finish_get_plan(snapshot):
    config = {"configurable": {"thread_id": "1"}}
    graph.invoke({"messages": snapshot.values["messages"],
                             "context": snapshot.values["context"],
                             "feedback": True
                            },
                            config)

def get_plan(situation = "Tell alice to hydrate.", last_prompt=None):

    context = "You are in the " + Config.current_room.name + ". "
    if Config.people_room_memory:
        context += "You remember that: "
        context += ", ".join([get_person_context(k, v) for k, v in Config.people_room_memory.items()])

    print(context)

    messages = [HumanMessage(content= f"# Goal: {situation}\n\nBehavior description:")]
    config = {"configurable": {"thread_id": "1"}}
    messages = graph.invoke({"messages": messages,
                             "context": context,
                             "feedback": False
                            },
                            config)
    
    snapshot = graph.get_state(config)

    json_plan = messages["messages"][-1].content
    return json_plan, snapshot

if __name__ == "__main__":
    prompt = input("Situation: ")
    plan = get_plan(prompt)
    print(f"LLM Plan:\n {plan}")
