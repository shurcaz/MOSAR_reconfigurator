import time

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D

from logic_layer.task_planner import Module, State, find_path, trim_state
from physical_layer.physical_layer import verify_pose, motion_planner
from test import *

# This program implements a simple Task and Motion Planner (TAMP) for the reconfiguration
# of modular blocks from a starting configuration to a goal configuration, under the physical
# constraints presented by a mobile manipulator rearranging modules on a desk

VIDEO_SAVE_LOCATION = __file__+"/../videos/"
VIDEO_FRAME_INTERVAL = 1000

STATIONARY_ARM_POSITION = [0.1, -0.3, 0]
MAX_PHYSICAL_LAYER_FAILURES = 500


# Create and save a video of a transition path
def create_reconfiguration_video(transition_path, label = "temp"):
    # find required frame size
    size = 0
    for state in transition_path:
        if state.size() > size:
            size = state.size()
    
    # Create figure
    fig = plt.figure(1)
    ax = plt.axes(111, projection = '3d')
    ax.set_title(label)
    ax.axis("off")
    
    # Create frames
    num_frames = len(transition_path)
    def update(frame):
        # Reset axis
        ax.clear()
        ax.set_title("Reconfiguration Video")
        ax.axis("off")
        
        # set the colors of each object
        current_state = transition_path[frame]
        colour_mask = current_state.create_colour_mask(size)
        
        container = ax.voxels(colour_mask,
                                facecolors = colour_mask,
                                edgecolors = 'k',
                                linewidth = 0.5)
        
        return container
    
    # Save Video
    ani = FuncAnimation(fig = fig, func = update, frames = num_frames, interval = VIDEO_FRAME_INTERVAL)
    ani.save(filename = VIDEO_SAVE_LOCATION + label + "/video.html", writer = "html")
    
    
# Verify each start/end movement position in a transition path is reachable
def verify_inverse_kinematics(transition_path):
    for state in transition_path:
        # if not start state
        if (state.birth_movement != 0):
            # get start/end positions in metres
            movement_decimetres = state.birth_movement
            movement_metres = np.divide(movement_decimetres, 10) # Convert to metres
            
            for pos in movement_metres:
                # verify arm can reach position
                if not verify_pose(STATIONARY_ARM_POSITION, pos):
                    return state
    return 0


# Generate a transition plan to reconfigure s_start into s_goal
def run_tamp_planner(s_start, s_goal):
    logic_time = 0
    physical_time = 0
    
    # Find semantic solution
    search_tree = None
    failure_count = 0
    while True:
        ##### Logic Layer #####
        t0 = time.time()
        transition_path, search_tree = find_path(s_start, s_goal, search_tree)
        logic_time += time.time() - t0
        
        # If logic layer failed, no solution
        if (transition_path == 0):
            print_invalid_sol(s_start.get_num_modules(), logic_time, physical_time)
            return 0, 0
        
        ##### Physical Layer #####
        t1 = time.time()
        
        # verify inverse kinematics
        failed_state = verify_inverse_kinematics(transition_path)
        
        # If IK failed, remove failed state and continue search
        if (failed_state != 0):
            if failure_count > MAX_PHYSICAL_LAYER_FAILURES:
                print_invalid_sol(s_start.get_num_modules(), logic_time, physical_time)
                return 0, 0
            trim_state(failed_state, search_tree)
            failure_count += 1
            physical_time += time.time() - t1
            continue
    
        # create motion plan
        instruction_set, failed_state = motion_planner(transition_path)
        
        # If motion plan failed, remove failed state and continue search
        if (failed_state != 0):
            if failure_count > MAX_PHYSICAL_LAYER_FAILURES:
                print_invalid_sol(s_start.get_num_modules(), logic_time, physical_time)
                return 0, 0
            trim_state(failed_state, search_tree)
            failure_count += 1
            physical_time += time.time() - t1
            continue
        else:
            # If motion plan successful
            physical_time += time.time() - t1
            break
            
    print_valid_sol(str(len(transition_path) - 1), failure_count, logic_time, physical_time)
    return instruction_set, transition_path


# Print Solution Found
def print_valid_sol(moves, failure_count, logic_t, physical_t):
        print("----- Solution Found -----")
        print("COUNT: moves              -", moves)
        print("COUNT: Failures           -", failure_count)
        print("TIME: Logic Layer time    -", logic_t, "s")
        print("TIME: Physical Layer time -", physical_t, "s")


# Print No Solution Found  
def print_invalid_sol(num_modules, logic_time, physical_time):
        print("----- NO Solution Found -----")
        print("COUNT: Modules            -", num_modules)
        print("TIME: Logic Layer time    -", logic_time, "s")
        print("TIME: Physical Layer time -", physical_time, "s")
    
    
def main():
    # Get test configurations
    s_start, s_goal, label = six_mod_config()
    
    # Run system
    instruction_set, transition_path = run_tamp_planner(s_start, s_goal)  
            
    # Display Output
    print("---------------------------------------------------------------------------")
    print("Instruction Set")
    for i in instruction_set:
        print(i)
    print("---------------------------------------------------------------------------")
    
    # Save video
    print("Video Name: " + label + " - " + str(len(transition_path) - 1) + " Moves")
    create_reconfiguration_video(transition_path, label + " - " + str(len(transition_path) - 1) + " Moves")
    print("Video Generated with", str(len(transition_path) - 1), "moves!")
   

if __name__ == "__main__":
    main()