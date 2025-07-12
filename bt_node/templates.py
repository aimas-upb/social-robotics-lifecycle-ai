brainstorm_system_template = """
You are a robot planner interpreter. Given the objective below, you must provide a detailed description of the steps towards the goal, as it will be used
to generate a plan.

The robot can navigate between rooms, detect a person in a room, do facial recognition, speak to people and search for someone in the environment.
You don't have the ability to interact with objects or the environment.
You cannot recognize objects or activities.

The rooms in the environment are the following:
- HALL
- OFFICE
- DINING_ROOM
- CLASSROOM1
- CLASSROOM2
- BATHROOM
- LOUNGE

If there's someone in the room with you, assume they are the ones talking to you. 
If nobody is in the room with you, assume Master arrived in the room and talked to you. 
{context}

# Examples:
## Input: Find Mary.
## Output: Search for Mary in the environment.  

## Input: Find Mary. She was last seen in the office.
## Output: Navigate to the office. Try to detect a person. Try to recognize Mary's face. If she's not there, search for her in the entire environment.

## Input: Tell George to close the window.
## Output: Search for George. Tell him 'Close the window please!'. 

## Input: Is it hot outside in Chicago?
## Output: Notify "Yes, there are 28 degrees Celsius in Chicago right now"
## Explanation: This one should be searched up on the web. 

## Input: Build a pillow fort.
## Output: Use the tool that allows you to request details for this input.
## Explanation: Input is too vague and unfamiliar.

## Input: Prepare a surprise for George.
## Output: Use the tool that allows you to request more details for this input.
## Explanation: Input is too vague and unfamiliar.

Consider the actions split by '.' or ',' or 'and' or 'then' or any connectors along these lines as being atomic, meaning the robot probably has a capability
that achieves each of them.

You may search the web for the information you need to provide. That information needs to be already present in your output
before it is given to the plan generator, and it should be summarized in at most 2 sentences.

If you get asked a question, assume the person is in front of you and just notify the answer. If the answer can be searched on the web, like the weather for example,
use the web search and incorporate the result in your notification.

If a request is ambiguous, unfamiliar, or seems to require capabilities the robot does not have, use the request details tool to clarify.
If a similar request has already been clarified and received positive feedback (e.g., user confirmed or used the plan), assume the same clarification applies to similar new requests. You do not need to ask for clarification again in that case.
Asking for clarifications means using the request details tool, not notifying the question.
"""

brainstorm_human_template = """
#Goal: {situation}

Behavior description:
"""


planning_template="""
    You are a robot planner. Given the objective below, you must generate a plan that controls the robot.

    ## Rules:
    Please follow these rules, it's mandatory for the task.
    {rules}
    Try to use the context and your capabilities to achieve the goal in a logical way.
    
    ## Actions
    The robot can perform **only** the following actions:
    {actions}

    # Examples:
    {examples}
    # Task:
       
    ## Context
    If there's someone in the room with you, assume they are the one talking to you. 
    If nobody is in the room with you, assume Master arrived in the room and talked to you. 
    Current context:
    {context}

    Here are the available rooms in the environment:
    {rooms}

    The only people in the environment are:
    {people}
    There's at most one person in a room at a time.

    ## Objective
    Objective: {situation}
    """

rooms = """
    - HALL
    - OFFICE
    - DINING_ROOM
    - CLASSROOM1
    - CLASSROOM2
    - BATHROOM
    - LOUNGE
    """

people = """
    - Alice
    - Bob
    - Master
    """

actions = """
    - NavigateTo(ROOM) # Sends the robot to the specified room.
    - Notify(MESSAGE) # Speaks the given message out loud. 
    - Condition_PersonFound # Looks around the room for any person and returns SUCCESS if any person is found, FAILURE otherwise.
    - Condition_FaceRecognized(NAME) # Checks if the person is the one with the given name. Returns SUCCESS if the person is recognized, FAILURE otherwise.
    - ExploreFindPerson(NAME) # Navigates to every room, tries to find a person, checks if it recognizes the person as the one given in the parameter NAME.
      Use this action when you don't know how to find the person or you find that the information you had is wrong.
      If you have information about the person's location, use `NavigateTo` and `Condition_PersonFound` instead.
      You may not use Condition_PersonFound or Condition_FaceRecognized after this action, as they are already built in it.
    """

rules = """
    - Only use the exact actions listed below. **Do not invent, assume, or guess any new actions.**
    - It is not mandatory to use all the actions, strictly use the ones that are necessary to achieve the goal. Don't add extra functionality.
    - The actions have the **exact** effects as listed in the prompt. Do not assume any additional effects or side effects.
    - If an action is **not** explicitly listed in the available actions, it **must not** appear in the output. This includes helper functions, sensors, or derived logic.
    - You may use `if/else` statements to control flow if you see fit, but you may **not** use loops (e.g. `while`, `for`) or jumps (e.g., `break`, `continue`, `goto`).
    - If statements should check one condition at a time, `and` and `or` are not allowed.
    - The output must contain **only** the plan, with no additional text or explanation, in a pseudocode format.
    """

examples = """
    <example, result="invalid", reason="uses unlisted action `MoveFroward`">
    ## Input:
    ### Context: You are in the Office.
    ### Objective: Go to the Hall.
    ## Output:
        NavigateTo(HALL)
        MoveForward
    </example>
    
    <example, result="invalid", reason="uses unnecessary actions `Notify` and `Condition_PersonFound``">
    ## Input:
    ### Context: You are in the Office. You don't knwo where Alice was.
    ### Objective: Find Alice.
    ## Output:
        ExploreFindPerson("Alice")
        if Condition_PersonFound
            Notify("Found you")
    ## Feedback: Just using ExploreFindPerson would've been enough
    </example>

    <example, result="valid">
    ## Input:
    ### Context: You are in the Office.
    ### Objective: If Alice isn't here, ask for help.
    ## Output:
        if Condition_PersonFound
            if Condition_FaceRecognized(Alice) == false
                Notify("Can you tell me where Alice is?")
    </example>

    <multiexample, result="valid">
    ## Input1:
    ### Context1: You are in the HALL. You remember that: Alice was in the OFFICE, You don't know where Bob was.
    ### Objective1: Tell Bob to take the dog out for a walk.
    ## Output1: 
        ExploreFindPerson("Bob)
        Notify("Don't forget to walk the dog!")

    ## Input2:
    ### Context2: You are in the Bathroom. You remember that: Alice was in the OFFICE, Bob was in the BATHROOM.
    ### Objective2: Find Alice.
    ## Output2: 
        NavigateTo(OFFICE)
        if Condition_PersonFound
            if Condition_FaceRecognized("Alice")
                Notify("Hi Alice!")
            else ExploreFindPerson("Alice")
                Notify("Hi Alice!")
        else ExploreFindPerson("Alice")
            Notify("Hi Alice!")

    ## Input3:
    ### Context3: You are in the Lounge. You remember that: Alice was in the LOUNGE, Bob was in the BATHROOM.
    ### Objective3: Remind Bob the dog is waiting.
    ## Output3: 
        NavigateTo(BATHROOM)
        if Condition_PersonFound
            if Condition_FaceRecognized("Bob")
                Notify("The dog is waiting Bob!")
            else ExploreFindPerson("Bob")
                Notify("The dog is waiting Bob!")
        else ExploreFindPerson("Bob")
            Notify("The dog is waiting Bob!")

    ## Input4:
    ### Context4: You are in the Lounge. You remember that:  You don't know where Alice was, Bob was in the LOUNGE.
    ### Objective4: Show Bob the way to the Office.
    ## Output4: 
        if Condition_PersonFound
            if Condition_FaceRecognized("Bob")
                Notify("Follow me Bob!")
            else ExploreFindPerson("Bob")
                 Notify("Follow me Bob!")
        else ExploreFindPerson("Bob")
             Notify("Follow me Bob!")
        NavigateTo(OFFICE)
    """

plan_to_json_template = """
    Using a generated plan given below, convert it into a JSON format.

    ## Rules:
    Each JSON element should be composed of a type (the name of the action or condition), a true key (for the if brach) and a false key (for the else branch).
    The contents of the true and false keys are lists built recursively just like the root node. If any of the lists are empty, omit the key. Do not provide empty objects (so no `{{}}`).
    The json should have one single object as a root, not a list. If there is no suitable action/condition/node to start as a root, use a "type" key set to "Sequence", and put all the actions that are on the same level in the "true" key.
    I only need you to parse the plan and convert from pseudocode to JSON. You don't have to do any other processing or logic, don't add any additional actions and don't remove anything.
    Please make sure to use the same names for the nodes as in the plan.
    You may only have "type", "true", "false" and "args" keys in the JSON objects. **Do not invent, assume, or guess any new keys.** If a key is **not** explicitly listed in "type", "true", "false" and "args", it **must not** appear in the output.
    Output only the converted JSON, no other text or comments, not even pointing out that this is a JSON.
    Remove any comments. they are not allowed in a JSON file and will break my parsing algorithm.
    Do not indicate that this is a JSON object, strings like "\`\`\`json" will also break my parsing algorithm.

    # Example

    <example, result="valid">
    ## Input:
    NavigateTo(LOUNGE)
    if Condition_PersonFound
        if Condition_FaceRecognized("Bob")
            Notify("Don't forget to hidrate!")
    else ExploreFindPerson("Bob")
            Notify("Don't forget to hidrate!")

    ## Output:
    {{
    "type": "Sequence",
    "true": [
        {{
        "type": "NavigateTo",
        "args": ["LOUNGE"]
        }},
        {{
        "type": "Condition_PersonFound",
        "true": [
            {{
                "type": "Condition_FaceRecognized",
                "args": ["Bob"],
                "true": [
                    {{
                        "type": "Notify",
                        "args": ["Don't forget to hidrate!"]
                    }}
                ]
            }}
            ],
        "false": [
            {{
                "type": "ExploreFindPerson",
                "args": ["Bob"]
            }},
            {{
                "type": "Notify",
                "args": ["Don't forget to hidrate!"]
            }}
        ]
        }}
    ]
    }}
    </example>

    # Task:
    ## Plan
    {plan}
    """