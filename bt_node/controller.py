from bt_node.langchain_planner import get_plan, finish_get_plan
from bt_node.behaviour_tree import BehaviorTreeRoot
import rclpy
from bt_node.config import Config
import heapq
import multiprocessing, itertools, time, threading
import sys

heap = [] 
heap_lock = multiprocessing.Lock()
goal_queue = multiprocessing.Queue()
counter = itertools.count() 
current_entry = None
current_bt = None

# One thread for user input
def input_thread():
    print("Hi! How may I assist you today?")

    snapshot = None

    while True:
        plan = None
        priority = 0
        user_input = input("Prompt: ")
        if user_input.startswith("test"):
            plan = None
        elif user_input.startswith("config"):
            print("Current configuration:")
            print(Config.people_rooms)
            print("Change people's locations in the building")

            person = input("Who do you want to move? (Alice/Bob/Master): ")
            for room in Config.Room:
                print(f"{room.name} = {room.value}")
            room_id = input("Where do you want to move them? (0-6): ")
            
            room = list(Config.Room)[int(room_id)]
            for k, v in Config.people_rooms.items():
                if v == person:
                    del Config.people_rooms[k]
                    Config.people_rooms[room] = person
                    print(f"Moved {person} to {room.name}")
                    break
            continue
        else:
            try:
                priority_str, actual_input = user_input.split("/", 1)
                priority = int(priority_str.strip())
                user_input = actual_input.strip()
                plan, snapshot = get_plan(user_input)
            except ValueError:
                print("Invalid format. Use format like: 1/Tell bob to hydrate")
                continue
     
        count = next(counter)

        start_room = Config.current_room
        new_entry = (priority, count, plan, start_room)

        with heap_lock:
            heapq.heappush(heap, new_entry)


# One thread for execution
def executor_thread():
    global current_entry, current_bt
    current_entry = None
    current_bt = None

    rclpy.init()
    while True:
        with heap_lock:
            # Check if a new higher-priority plan is in the queue
            if heap:
                if (current_entry is None) or (current_entry[0] > heap[0][0]):
                    # Stop current BT and put it back
                    if current_bt:
                        current_bt.done = True
                        current_bt.destroy_node()
                        heapq.heappush(heap, current_entry)
                    
                    current_entry = heapq.heappop(heap)
                    current_bt = BehaviorTreeRoot(plan=current_entry[2], start_room=current_entry[3])
        
        if current_bt:
            rclpy.spin_once(current_bt, timeout_sec=0.1)
            if getattr(current_bt, 'done', True):
                current_bt.destroy_node()
                current_bt = None
                current_entry = None

    rclpy.shutdown()

def main_no_preempt(args=None):
    print("Hi! How may I assist you today?")

    snapshot = None

    while True:
        user_input = input("Prompt: ")

        plan = None
        if user_input.startswith("test"):
            plan = None
        elif user_input.startswith("config"):
            print("Current configuration:")
            print(Config.people_rooms)
            print("Change people's locations in the building")

            person = input("Who do you want to move? (Alice/Bob/Master): ")
            for room in Config.Room:
                print(f"{room.name} = {room.value}")
            room_id = input("Where do you want to move them? (0-6): ")
            
            room = list(Config.Room)[int(room_id)]
            for k, v in Config.people_rooms.items():
                if v == person:
                    del Config.people_rooms[k]
                    Config.people_rooms[room] = person
                    print(f"Moved {person} to {room.name}")
                    break
            continue
        else:
            plan, snapshot = get_plan(user_input)

        rclpy.init(args=args)
        node = BehaviorTreeRoot(plan=plan)

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if getattr(node, 'done', False):
                break

        node.destroy_node()
        rclpy.shutdown()

        if plan:
            finish_get_plan(snapshot)


def main():
    preempt = True
    if len(sys.argv) > 1 and sys.argv[1].lower() == "false":
        preempt = False
    
    if preempt:
        thread1 = threading.Thread(target=input_thread)
        thread2 = threading.Thread(target=executor_thread)

        thread1.start()
        thread2.start()

        thread1.join()
        thread2.join()
    else:
        main_no_preempt()

if __name__ == '__main__':
    main()