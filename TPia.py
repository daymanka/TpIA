
from collections import deque
import heapq


class Bucket:
    def __init__(self, capacity, water_level=0):
        self.capacity = capacity
        self.water_level = water_level

    def fill(self):
        self.water_level = self.capacity

    def empty(self):
        self.water_level = 0

    def pour(self, other_bucket):
        if other_bucket.water_level < other_bucket.capacity:
            transfer_amount = min(self.water_level, other_bucket.capacity - other_bucket.water_level)
            self.water_level -= transfer_amount
            other_bucket.water_level += transfer_amount
            return True
        return False

"""
def depth_first_search(state, goal_state, buckets):
    visited_states = set()
    stack = [(state, [])]
    print(stack)

    while stack:
        current_state, path = stack.pop()
        print(current_state)
        print(path)
        if current_state == tuple(goal_state):
            print("voici le chemin")
            print(visited_states)
            return path
        

        visited_states.add(current_state)
        for action in generate_actions(current_state, buckets):
            next_state = apply_action(current_state, action, buckets)
            if next_state not in visited_states:
                stack.append((next_state, path + [action]))

    return visited_states
"""
def depth_first_search(initial_state, goal_state, buckets):
    open_set = [initial_state]
    closed_set = set()

    while open_set:
        current_state = open_set.pop()
        closed_set.add(current_state)
        
        if current_state == goal_state:
            return True  # But atteint
        
        closed_set.add(current_state)
        for action in generate_actions(current_state, buckets):
            next_state = apply_action(current_state, action, buckets)
            if next_state not in open_set and next_state not in closed_set:
                open_set.append(next_state)
    print(open_set)
    print(closed_set)
    
    
    return False  # But non atteint



# Exemple d'utilisation :
# définir la fonction est_un_but, generer_voisins et le départ
# trouver_but(depart, est_un_but, generer_voisins)

def generate_actions(state, buckets):
    actions = []
    num_buckets = len(buckets)
    
    # Transvasement de si vers sj
    for i in range(num_buckets):
        for j in range(num_buckets):
            if i != j and state[i] > 0 and state[j] < buckets[j].capacity:
                actions.append(('pour', i, j))
    
    # Transvasement de sj vers si
    for i in range(num_buckets):
        for j in range(num_buckets):
            if i != j and state[i] < buckets[i].capacity and state[j] > 0:
                actions.append(('pour', j, i))
    
    # Actions de remplissage et de vidage
    for i in range(num_buckets):
        if state[i] < buckets[i].capacity:  # Vérifier si le seau n'est pas plein
            actions.append(('fill', i))
        if state[i] > 0:  # Vérifier si le seau n'est pas vide
            actions.append(('empty', i))
     
    return actions


def apply_action(state, action, buckets):
    new_state = list(state)
    action_type = action[0]
    if action_type == 'fill':
        bucket_index = action[1]
        new_state[bucket_index] = buckets[bucket_index].capacity
    elif action_type == 'empty':
        bucket_index = action[1]
        new_state[bucket_index] = 0
    elif action_type == 'pour':
        source_index, target_index = action[1], action[2]
        buckets[source_index].pour(buckets[target_index])
        new_state[source_index] = buckets[source_index].water_level
        new_state[target_index] = buckets[target_index].water_level
    return tuple(new_state)




def breadth_first_search(state, goal_state, buckets):
    visited_states = set()
    queue = deque([(state, [])])

    while queue:
        current_state, path = queue.popleft()
        if current_state == goal_state:
            return path

        visited_states.add(current_state)
        for action in generate_actions(current_state, buckets):
            next_state = apply_action(current_state, action, buckets)
            if next_state not in visited_states:
                queue.append((next_state, path + [action]))

    return None


def best_first_search(state, goal_state, buckets, heuristic):
    visited_states = set()
    priority_queue = [(heuristic(state, goal_state), state, [])]

    while priority_queue:
        _, current_state, path = heapq.heappop(priority_queue)
        if current_state == goal_state:
            return path

        visited_states.add(current_state)
        for action in generate_actions(current_state, buckets):
            next_state = apply_action(current_state, action, buckets)
            if next_state not in visited_states:
                heapq.heappush(priority_queue, (heuristic(next_state, goal_state), next_state, path + [action]))

    return None

def simple_heuristic(state, goal_state):
    return sum(abs(state[i] - goal_state[i]) for i in range(len(state)))


def read_instance(file_path):
    with open(file_path, 'r') as file:
        num_buckets = int(file.readline().strip())
        capacities = list(map(int, file.readline().strip().split()))
        water_levels = list(map(int, file.readline().strip().split()))
    return num_buckets, capacities, water_levels


def solve_instance(file_path, algorithm):
    num_buckets, capacities, goal_state = read_instance(file_path)
    initial_state = tuple([0] * num_buckets)
    buckets = [Bucket(capacity,0) for capacity in capacities]
    
    print(num_buckets)
    print(initial_state)
    print(capacities)
    print(goal_state)


    if algorithm == 'dfs':
        solution = depth_first_search(initial_state, goal_state, buckets)
        
    elif algorithm == 'bfs':
        solution = breadth_first_search(initial_state, goal_state, buckets)
    elif algorithm == 'best_first':
        solution = best_first_search(initial_state, goal_state, buckets, simple_heuristic)
    else:
        raise ValueError("Invalid algorithm specified")

    return solution

file_path= "td.txt"

file_path1="3_7_13_19_23_29_a.buck"
path3="6_10_15_a.buck"
solution=solve_instance(file_path, "dfs")
print(solution)