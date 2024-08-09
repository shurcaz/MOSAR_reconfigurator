import math
from .utils import *

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# This logic layer file details a simple Task Planner to generate semantic solutions 
# for the reconfiguration of a start state to a goal state


MAXIMUM_BRANCHES = 5


# Generate priority queue of state transitions
def generate_states(state, s_goal):
    states_queue = StateQueue(s_goal)
    
    # Get available positions for modules to move to
    available_positions = state.get_available_positions()
                
    # Generate movement for each movable module in non_final position
    generated_moveset = state.generate_moves(state.get_non_final_positions(s_goal), available_positions)
    states_queue.push_multiple(generated_moveset)

    # If no new state generated, allow movement of adjacent final modules
    if states_queue.is_empty():
        adjacent_modules = []
        for module_pos in state.get_non_final_positions(s_goal):
            adjacent_modules = adjacent_modules + state.get_adjacent_modules(module_pos)
        adjacent_modules = np.unique(adjacent_modules, axis=0)
        
        # Generate movement for each adjacent module to modules in non_final position
        generated_moveset = state.generate_moves(adjacent_modules, available_positions)
        states_queue.push_multiple(generated_moveset)
    
    
    # If no new state generated, allow movement of all modules
    if states_queue.is_empty():
        # Generate movement for all modules
        generated_moveset = state.generate_moves(state.get_module_positions(), available_positions)
        states_queue.push_multiple(generated_moveset)

    return states_queue


# Create a task plan to reconfigure state s_start into start s_goal
# If a search tree is input, planner continues search through search tree
def find_path(s_start, s_goal, search_tree = None):
    if search_tree == None:
        search_tree = []
    
    # Confirm contents of each state match
    if not do_contents_match(s_start, s_goal):
        print("ERROR: Start and Goal state do not have an equal composition of modules")
        return 0
    
    # Set the recursion limit according to the input state
    recursion_limit = set_recursion_limit(s_start)
    
    recursion_counter = 1
    s_current = s_start
    
    # While goal state not found
    while not s_current.equals(s_goal):
        # generate state children
        priority_queue_new = generate_states(s_current, s_goal)
        
        # add children to list
        for i in range(MAXIMUM_BRANCHES):
            if (priority_queue_new.get_length()):
                s_new = priority_queue_new.pop()
                s_new.parent = s_current
                search_tree.append(s_new)
        
        # Get next state
        if (recursion_counter < recursion_limit):
            s_current = search_tree.pop(0)
            recursion_counter += 1
        else:
            print("MAX RECURSION COUNT REACHED! UNABLE TO FIND PATH.")
            return 0, 0
    
    # when goal reached, return transition plan and current search tree
    return s_current.get_state_path(), search_tree


# Test whether states have equal module compositions
def do_contents_match(s_start, s_goal):
    # Confirm number of modules match
    if s_start.get_num_modules() != s_goal.get_num_modules():
        return False
    
    # Confirm composition of modules match
    unmatched = np.asarray(s_goal.get_modules())
    for m1 in np.asarray(s_start.get_modules()):
        for m2 in unmatched:
            # remove all module in s_goal from unmatched list
            if Module.equals(m1, m2):
                unmatched = unmatched[unmatched != m2]
    
    # If unmatched array is empty, states are made up of same modules
    return not len(unmatched)
        

# Return a sensible task planner recursion limit for the number of modules in a state
# Returns enough recursions for a reconfiguration plan with (1.5 * number of modules) moves
def set_recursion_limit(state):
    return math.pow(MAXIMUM_BRANCHES, len(state.get_modules()) * 1.5)


# Remove a state and all states generated from the removed state from the search tree
def trim_state(state, search_tree):
    trimmed_search_tree = []
    for s in search_tree:
        if state not in s.get_state_path():
            trimmed_search_tree.insert(0, trimmed_search_tree)
    return trimmed_search_tree
            
        
# An implementation of a state priority queue
# where priority is defined by similarity to a goal state
class StateQueue:
    def __init__(self, s_goal):
        self.queue = []
        self.s_goal = s_goal
        
        
    def __str__(self):
        s = "----- State Queue ----- "
        s += "\nLength: " + str(self.get_length())
        count = 1
        for state in self.queue:
            s += "\nItem" + str(count) + ":"
            s += "\n\tFinal modules: " + str(State.get_num_modules_in_final_position(state, self.s_goal))
            s += "\n\tFree modules: " + str(State.get_num_modules_in_free_position(state, self.s_goal))
            s += "\n\tDistance modules: " + str(State.get_distance_from_completion(state, self.s_goal))
            count += 1
        return s
    
    
    # Return number of states in the queue
    def get_length(self):
        return len(self.queue)
    
    
    # Return True if queue is empty, else False
    def is_empty(self):
        return len(self.queue) == 0
    
    
    # Remove and return a state from the front of the queue
    def pop(self):
        return self.queue.pop(0)
    
    
    # Insert a state into the queue at an index that maintains priority order
    def push(self, state):
        self.queue.insert(self.binary_search(state), state)
    
    
    # Insert an array of states to the queue
    def push_multiple(self, state_array):
        for state in state_array:
            self.push(state)
    
    
    # Find a states position in the queue according to priority order
    def binary_search(self, state):
        if len(self.queue) == 0:
            return 0
            
        # Use binary search to find state position in queue
        high = len(self.queue) - 1
        low = 0
        
        while (low <= high):
            mid = math.floor((high + low) / 2)
            comparison_val = self.comparator(state, self.queue[mid])
            if ( comparison_val == 0):
                return mid
            elif ( comparison_val > 0 ):
                # if inserting state higher priority, move search to left of queue
                high = mid - 1
            else:
                # if inserting state lower priority, move search to right of queue
                low = mid + 1
        return low


    # Return
    # POSITIVE if state_1 priority is HIGHER than state_2
    # ZERO     if state_1 priority is EQUAL to state_2
    # NEGATIVE if state_1 priority is LOWER than state_2
    def comparator(self, state_1, state_2):
        # if state_1 has higher number of modules in final position, return positive else negative
        s1_finalist_num = State.get_num_modules_in_final_position(state_1, self.s_goal)
        s2_finalist_num = State.get_num_modules_in_final_position(state_2, self.s_goal)
        if (s1_finalist_num != s2_finalist_num):
            return s1_finalist_num - s2_finalist_num
        
        # if equal, return state with highest number of modules not in final position, but are in free position
        # if state_1 has higher number of modules not in final pos but in free pos, return positive
        s1_free_num = State.get_num_modules_in_free_position(state_1, self.s_goal)
        s2_free_num = State.get_num_modules_in_free_position(state_2, self.s_goal)
        if (s1_free_num != s2_free_num):
            return s1_free_num - s2_free_num
        
        # if equal finalists and free modules, return state with lowest
        # sum distance between free modules and final locations
        # if state_1 distance is lower, return positive
        s1_dist = State.get_distance_from_completion(state_1, self.s_goal)
        s2_dist = State.get_distance_from_completion(state_2, self.s_goal)
        if (s1_dist != s2_dist):
            return s2_dist - s1_dist
        
        # Return equal priority
        return 0


# An implementation of a state configuration and associated functions
class State:
    def __init__(self):
        self.parent = 0
        self.birth_movement = 0
        self.modules_dict = dict()
        
        # values for state queue comparison
        self.comparison_goal_state = None
        self.finalist_val = None
        self.free_val = None
        self.goal_distance = None
    
    
    # Get number of modules in the state
    def get_num_modules(self):
        return len(self.get_modules())
        
    
    # Return a copy of the current state
    def duplicate(self):
        duplicate = State()
        duplicate.modules_dict = self.modules_dict.copy()
        return duplicate
        
    
    # Set a goal state for similarity measurement calculations
    def set_goal(self, goal_state):
        self.comparison_goal_state = goal_state
        self.finalist_val = None
        self.free_val = None
        self.goal_distance = None

    
    # Reset saved similarity measurements
    def reset_saved_values(self):
        self.finalist_val = None
        self.free_val = None
        self.goal_distance = None

    
    # Insert a module into the state at position
    def insert(self, position, module):          
        position = tuple(position)
        if position not in self.modules_dict:
            self.modules_dict[position] = module
        self.reset_saved_values()
        
    
    # Remove module from state from position
    def remove_module(self, position):
        self.reset_saved_values()
        return self.modules_dict.pop(position)
    
    
    # Move a module from start_pos to end_pos, if movement is valid
    # Returns a copy of current state with module in new position
    def move_module(self, start_pos, end_pos):
        state = self.duplicate()
        mod = state.remove_module(start_pos)
        t1 = state.is_connected()
        state.insert(end_pos, mod)
        t2 = state.is_connected()
        return state if (t1 and t2) else 0
    
    
    # Return true if 2 states are equal
    def equals(self, compared_state):
        for pos in self.get_module_positions():
            if ((pos not in compared_state.modules_dict) or 
                (not Module.equals(self.modules_dict[pos], compared_state.modules_dict[pos]))):
                return False
        return True
    
    
    # Get size of the state (maximum 3D length)
    def size(self):
        return np.max(list(self.modules_dict.keys())) + 1
    
    
    # Return a position matrix of the current state
    def to_position_matrix(self, min_size = 0):
        positions = list(self.modules_dict.keys())
        max_val, min_val = max_min(positions)
        
        # shift module positions to move negative positions into positive space
        adjustment_val = abs(min_val) if min_val < 0 else 0
        
        # find required size of matrix
        max_val += 1
        size = max_val if max_val > min_size else min_size
        size += adjustment_val
        
        # Copy dictionary to position matrix
        position_matrix = np.zeros((size, size, size), dtype = 'object')
        for pos in positions:
            pos_adjusted = increment_tuple(pos, adjustment_val)
            position_matrix[pos_adjusted] = self.modules_dict[pos]
        return position_matrix
    
    
    # Create a colour mask for state visualisation
    def create_colour_mask(self, min_size = 0):
        # Get position matrix
        colour_mask = self.to_position_matrix(min_size)
        
        # Replace each module in matrix with the module colour
        it = np.nditer(colour_mask, flags=['multi_index', 'refs_ok'])
        for x in it:
            if x.item() != 0:
                colour_mask[it.multi_index] = colour_mask[it.multi_index].colour
        return colour_mask
        
    
    # Create and display a figure of the current state
    # Argument label is the displayed figure title
    def display(self, label = ""):
        # Get colour mask
        colour_mask = self.create_colour_mask()
        
        # plot state
        plt.figure()
        ax = plt.axes(projection = '3d')
        ax.set_title(label)
        ax.axis("off")
        
        # Display mask
        container = ax.voxels(colour_mask,
                                facecolors = colour_mask, 
                                edgecolors = 'k',
                                linewidth = 0.5)        
        
        plt.show(block=True)
        return container
    
    
    # Validate whether all modules in a state are connected
    # Returns True if all modules are connected, else False
    def is_connected(self):     
        processed_modules = []
        discovered_modules = []
        
        # Add first module in dictionary to discovered modules
        discovered_modules.append(list(self.modules_dict.keys())[0])
        
        # Process all discovered modules
        while len(discovered_modules):
            mod = discovered_modules.pop()
            # Get neighbours
            adjacent_modules = self.get_adjacent_modules(mod)
            
            # If neighbour has not been discovered yet, add to discovered modules
            for adj_mod in adjacent_modules:
                if (adj_mod not in processed_modules) and (adj_mod not in discovered_modules):
                    discovered_modules.append(adj_mod)
            processed_modules.append(mod)
        
        # if number of discovered modules matches number of module in the state, return True
        return len(processed_modules) == len(self.modules_dict)
    
    
    # Verify a module can be connected to
    # Return true if module has an exposed face, else False
    def is_module_removable(self, pos):      
        # if an adjacent position doesn't contain a module
        for i in State.get_adjacent_positions(pos):
            if i not in self.modules_dict:
                # module has free face to connect to
                return True
        return False
    
    
    # Return state transition history
    def get_state_path(self):
        task_plan_list = [self]

        while task_plan_list[0].parent != 0:
            task_plan_list.insert(0, task_plan_list[0].parent)
            
        return task_plan_list
        
    
    # Return array of modules in the state
    def get_modules(self):
        return list(self.modules_dict.values())
    
    # Return array of module positions in the state
    def get_module_positions(self):
        return list(self.modules_dict.keys())
            
    
    # Get number of modules in the current state that match positions in the goal state
    def get_num_modules_in_final_position(self, goal_state):
        # if value already computed, return
        if self.comparison_goal_state == goal_state and self.finalist_val != None:
            return self.finalist_val
        
        if self.comparison_goal_state != goal_state:
            self.set_goal(goal_state)
        
        count = 0
        
        for position in self.get_module_positions():
            # if position is occupied in both states
            if position in goal_state.modules_dict:
                #if position is occupied with equal modules
                if Module.equals(self.modules_dict[position], goal_state.modules_dict[position]):
                    count += 1
                
        self.finalist_val = count
        return self.finalist_val
    
    
    # Get number of modules in the current state that do not match positions in the goal state
    def get_non_final_positions(self, goal_state):
        results = []
        
        # if position not occupied
        # or
        # if modules don't match
        for position in self.get_module_positions():
            if position not in goal_state.modules_dict:
                results.append(position)
            else:
                if not Module.equals(self.modules_dict[position], goal_state.modules_dict[position]):
                    results.append(position)
                
        return results
    
    
    # Get number of modules in the current state that do not match positions in the goal state,
    # and are in positions that are not occupied in the goal state
    def get_num_modules_in_free_position(self, goal_state):
        # if value already computed, return
        if self.comparison_goal_state == goal_state and self.free_val != None:
            return self.free_val
        
        if self.comparison_goal_state != goal_state:
            self.set_goal(goal_state)
            
        count = 0
        for position in self.get_module_positions():
            if position not in goal_state.modules_dict:
                count += 1
                
        self.free_val = count
        return self.free_val
    
    
    # Get euclidean distance of all non-final modules to their final positions
    def get_distance_from_completion(self, goal_state):
        # if value already computed, return
        if self.comparison_goal_state == goal_state and self.goal_distance != None:
            return self.goal_distance
        
        if self.comparison_goal_state != goal_state:
            self.set_goal(goal_state)
            
        # get modules positions not in final pos
        # get empty final module locations
        non_final_module_positions = self.get_non_final_positions(goal_state)
        empty_final_positions = []
        
        # if position not occupied
        # or
        # if modules don't match
        for position in goal_state.get_module_positions():
            if not ((position in self.modules_dict) and 
                    (Module.equals(goal_state.modules_dict[position], self.modules_dict[position]))):
                empty_final_positions.append(position)
        
    
        # for each module not in final pos, get sum of distances to empty final locations
        dist = 0
        for p1 in non_final_module_positions:
            for p2 in empty_final_positions:
                dist += np.linalg.norm(np.asarray(p1) - np.asarray(p2))
                
        self.goal_distance = dist
        return self.goal_distance
    
    
    # Get available positions in the state that can be connected to
    def get_available_positions(self):
        available_positions = []
        
        # iterate through modules and add all free positions to list
        for module_pos in self.get_module_positions():
            for adjacent_pos in State.get_adjacent_positions(module_pos):
                if adjacent_pos not in self.modules_dict:
                    available_positions.append(adjacent_pos)
        
        # Remove duplicates and return positions list
        return np.unique(available_positions, axis=0)
        
    
    # Get adjacent modules to module in position pos
    def get_adjacent_modules(self, pos):
        adjacent_modules = []
        for adjacent_pos in State.get_adjacent_positions(pos):
            if adjacent_pos in self.modules_dict:
                adjacent_modules.append(adjacent_pos)
        return adjacent_modules
    
    
    # Get adjacent positions to position
    def get_adjacent_positions(position):
        # get adjacent positions
        adjacent_positions = []
        for i in range(0,3):
            adjacent_positions.append(increment_tuple_val(position, i, 1))
            adjacent_positions.append(increment_tuple_val(position, i, -1))
        return adjacent_positions
    
    
    # Generate a new state for each valid movement of each module in module_positions to each movement_position
    # Return generated states as a list
    def generate_moves(self, module_positions, movement_positions):
        moveset = []
        for module_pos in module_positions:
            # If module has an available connection point to be grabbed by a manipulator
            if (self.is_module_removable(module_pos)):
                # if state valid without module
                tmp = self.duplicate()
                mod = tmp.remove_module(tuple(module_pos))
                
                if tmp.is_connected():
                    # For each available position, generate a valid state with the possible movement
                    for move_pos in movement_positions:
                        return_state = tmp.duplicate()
                        return_state.insert(tuple(move_pos), mod)
                        if return_state.is_connected():
                            return_state.birth_movement = [module_pos, move_pos]
                            moveset.append(return_state)
        return moveset
        
    
# An implementation of a basic Module object
class Module:
    id_number = 0
    module_type = 'basic'
    
    # Initialize module with identifying colour
    def __init__(self, colour):
        self.colour = colour
        
        
    def __str__(self):
        return f"""Module - {self.id_number}: \n\tColour: {self.colour}\n\tType: {self.module_type}"""
    
    
    # Return true if 2 modules are of equal colour
    def equals(mod_1, mod_2):
        return (mod_1 != 0 and 
                mod_2 != 0 and
                mod_1.colour == mod_2.colour)
        
    # Set module colour
    def set_colour(self, colour):
        self.colour = colour
        
        
