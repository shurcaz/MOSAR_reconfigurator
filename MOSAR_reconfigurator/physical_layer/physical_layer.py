import ikpy.chain as ik
import ikpy.utils.plot as plot_utils

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from logic_layer.task_planner import State
import numpy as np


# This physical layer file details a simple inverse kinematics verifier and motion planner for the 
# verification of Logic Layer semantic solutions and the generation of mobile manipulator instructions sets


# Import mobile manipulator specifications
my_chain = ik.Chain.from_urdf_file("physical_layer/arm_urdf.urdf", active_links_mask=[False, True, True, True, True, True, True])
HOME_POSITION_ANGLES = [0, 0, 1, -2.5, 0, -1.6, 0]
END_POSITION_ERROR_MARGIN = 0.0015 # 1.5 mm


#verify if a position is reachable from an arm positioned at arm_position
def verify_pose(arm_position, target_position):
    target_position -= arm_position     # arm base offset
    target_position += [0.05, 0.05, 0.1]    # target middle of block top face

    # orientate end-effector downwards
    target_orientation = [0, 0, -1]
    orientation_mode = "Z"

    # Get joint angles to place end-effector at target_position at the target_orientation
    ik = my_chain.inverse_kinematics(target_position, target_orientation, initial_position=HOME_POSITION_ANGLES, orientation_mode=orientation_mode)
    
    # Verify joint angles with forward kinematics
    computed_position = my_chain.forward_kinematics(ik)

    # If end-effector within error margin of target position, return true
    if is_valid_position(computed_position[:3, 3], target_position):
        return True
    else:
        return False


# Get joint angles to place end-effector at target_position orientated downwards
def get_ik(target_position):
    # point down
    target_orientation = [0, 0, -1]
    orientation_mode = "Z"

    # Get joint angles
    return my_chain.inverse_kinematics(target_position, target_orientation, initial_position=HOME_POSITION_ANGLES, orientation_mode=orientation_mode)


# Return true if position is within error range of target position, else False
def is_valid_position(position, target_position):
    error_margin = END_POSITION_ERROR_MARGIN
    for i in range(0,3):
        if (position[i] < target_position[i] - error_margin) or (position[i] > target_position[i] + error_margin):
            return False
    return True


# Display mobile manipulator pose
def display_pose(inverse_kinematics, target_position):
    fig, ax = plot_utils.init_3d_figure()
    fig.set_figheight(9)  
    fig.set_figwidth(13)  
    my_chain.plot(inverse_kinematics, ax, target=target_position)
    plt.xlim(-0.5, 0.5)
    plt.ylim(-0.5, 0.5)
    ax.set_zlim(0, 0.6)
    plt.ion()
    plt.show()
    

# Verify transition path movements adhere to physical constraints
# If they do, Return list of instructions for the mobile manipulator
def motion_planner(transition_path):
    # confirm all movements possible
    for state in transition_path:
        if (state.birth_movement != 0):
            if not movement_possible(state):
                return 0, state
    
    
    # develop movement plan
    # instructions entered according to key skills:
    # ["CONNECT"]
    # ["DISCONNECT"]
    # ["MOVE_TO", target_pos, joint_angles_array]
    instruction_set = [["START"]]
    for state in transition_path:
        if (state.birth_movement != 0):
            instruction_set.append(generate_move_inst(state.birth_movement[0]))
            instruction_set.append(["CONNECT"])
            
            instruction_set.append(generate_move_inst(state.birth_movement[1]))
            instruction_set.append(["DISCONNECT"])
    instruction_set.append(["END"])
    
    return instruction_set, 0
    

# Return move to instruction for position
def generate_move_inst(position):
    # correct module position to top face of module
    target_position = np.asarray(position) + [0.05, 0.05, 0.1]
    return ["MOVE_TO", tuple(target_position), tuple(get_ik(target_position))]
    

# Verify movement adheres to physical constraints
def movement_possible(state):
    movement = state.birth_movement
    # verify module can be lifted (no blocks above module)
    for key in state.get_module_positions():
        if (key[0] == movement[0][0]) and (key[1] == movement[0][1]) and (key[2] > movement[0][2]):
            return False
    
    # verify module placement is above surface
    if movement[1][2] < 0:
        return False
    
    # verify module placement is supported position
    # (a surface exists directly below placement position)
    supporting_position = tuple(movement[1] - [0,0,1])
    if (movement[1][2] != 0):
        if not (supporting_position in state.get_module_positions()):
            return False
        
    return True