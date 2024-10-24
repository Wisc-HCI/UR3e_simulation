import lively
from lively import Solver, Translation, Rotation,Transform,SmoothnessMacroObjective, CollisionAvoidanceObjective, JointLimitsObjective, BoxShape, CollisionSettingInfo, PositionMatchObjective, OrientationMatchObjective, ScalarRange, JointMatchObjective, State
from lxml import etree
import numpy as np
import pandas as pd
#import roboticstoolbox as rtb
#import spatialmath as sm
#from swift import Swift
#import json
#import os
#import csv
#import socket
import time
#from csv import reader
#import math
#import datetime
#import sys
#import contextlib
#import io
## local imports
from pose_reached import poseReached, orientationReached
#from Lib.joint_reached import jointsReached, jointReached
from Pose import Pose
## critic imports
# from critic_end_effector_pose import evaluate as critic_end_effector_pose
# from critic_space_usage import evaluate as critic_space_usage
# from critic_pinch_points import evaluate as critic_pinch_points
# from critic_joint_speed import evaluate as critic_joint_speed
# Constructed CollisionAvoidance Objective ,BoxShape and configured Collision Setting
# box = BoxShape(name="Table",frame="world",physical=True,x=2,y=1,z=1.2,translation = Translation(x=1.0, y =2.0, z=3.0),
# rotation = Rotation(x=0.0,y=0.0,z=0.0,w=1.0))
# collision = CollisionSettingInfo(d_max = 0.1, r = 0.0, a_max = 2.0, time_budget = 100, timed = False)

########### connection to the physical ur3e robot
INTERVAL_TIME = 45

class FunctionLib:

    def __init__(self):     
        # Read the xml file into a string
        # self.xml_file = './Lib/panda.xml'
        self.xml_file = 'ur3.xml'
        tree = etree.parse(self.xml_file)
        self.xml_string = etree.tostring(tree).decode()
        self.initial_state = State(origin=Transform(Translation(x=0.0, y=0, z=0.59), Rotation(x=0.0, y=0.0, z=1.0, w=0.0)), joints={
            "shoulder_pan_joint": 0,
            "elbow_joint": 0,
            "wrist_3_joint": 1,
            "shoulder_lift_joint": 0,
            "wrist_1_joint": 0,
            "wrist_2_joint": 0,
        })
        self.objectives = {
            "smoothness":SmoothnessMacroObjective(name="MySmoothnessObjective",weight=10.0,joints=True,origin=False,links=True),
            "collision": CollisionAvoidanceObjective(name="MyCollisionAvoidanceObjective", weight=3.0),
            "positionMatch":PositionMatchObjective(name="MyPositionMatchObjective", link="gripper_offset", weight=15.0),
            "orientationMatch":OrientationMatchObjective(name="MyOrientationMatchObjective", link="gripper_offset", weight=10.0),
            "jointMatch": JointMatchObjective(name="MyJointMatchObjective", joint = "robotiq_85_left_knuckle_joint", weight=20.0),           
        }
        self.root_bounds = [
            ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0,delta=0.0),ScalarRange(value=0.59,delta=0.0), # Translational, (x, y, z)
            ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0),ScalarRange(value=3.141592653589793,delta=0.0)  # Rotational, (r, p, y)
        ]
        self.shapes = [
            BoxShape(name="Table",frame="world",physical=True,x=1.2,y=0.6,z=0.05,translation = Translation(x=0.0, y =0.36, z=0.74), rotation = Rotation(x=0.0,y=0.0,z=0.0,w=1.0)),
            # BoxShape(name="red_block",frame="world",physical=True,x=0.04,y=0.15,z=0.08,translation = Translation(x=0.0, y =0.25, z=0.88), rotation = Rotation(x=0.0,y=0.0,z=0.0,w=1.0))
        ]
        
        # Instantiate a new solver
        self.solver = Solver(
        urdf=self.xml_string, # Full urdf as a string
        objectives = self.objectives,
        root_bounds=self.root_bounds,
        shapes=self.shapes,
        initial_state=self.initial_state,
        # collision_settings = collision
        )  
        self.trajectory = [] # list of robot states added with time key value
        self.robot_states = [] # list of robot states changed to tfs with state2tfs function
        self.joint_states = [] # list of joint states
        self.states_with_time = [] # list of states added with time key value
        self.goals = {}
        # self.goals["positionMatch"] = Translation(-0.35289461347986373, -0.01068364098, 1.14946876678037234)
        self.goals["orientationMatch"] = Rotation(x=0, y=1, z=0, w=0) 
        #self.target_position = (0.0, 0.0, 0.0)
        self.poseReached = False
        self.orientationReached = False
        self.jointReached = False 
        
        # for i in range(100):
        #     self.solver.solve(self.goals, weights = {},time = 0.0)
        # self.solve_state = self.solver.solve(self.goals, weights = {},time = 0.0)
          
    def move_to(self, x, y, z):
        # Initialize the time variable
        time = 0.0  
        # Initialize the poseReached variable
        self.poseReached = False
        # if not moving to the first waypoint
        if(self.trajectory != []):     
            # update weights and goals
            self.objectives["jointMatch"] = JointMatchObjective(name="MyJointMatchObjective", joint = "wrist_3_joint", weight=0.0)
            self.objectives["positionMatch"] = PositionMatchObjective(name="MyPositionMatchObjective", link="wrist_3_joint", weight=30.0)
            # initialize updated solver
            self.solver = Solver(
            urdf=self.xml_string, # Full urdf as a string
            objectives=self.objectives,
            initial_state = State(origin=Transform(Translation(x=0.0, y=0, z=0), Rotation(x=0.0, y=0.0, z=1.0, w=0.0)), joints=self.trajectory[-1]["joints"]),
            root_bounds=self.root_bounds,
            shapes = self.shapes
            )
            # update time variable
            time = self.trajectory[-1]["time"] + INTERVAL_TIME
        
        # List to collect the states
        joint_states = []
        robot_states = []
        states = []
        joint_state_copy = {}
        # target_position = self.lively_position_transform(x, y, z)
        self.target_position = (x, y, z)
        print("goal is: ",self.target_position)
        self.goals["positionMatch"] = Translation(self.target_position[0], self.target_position[1], self.target_position[2])
        max_iterations = 1000
        i = 0

        while not self.poseReached:
            state = self.solver.solve(self.goals, weights = {},time = 0.0)
            p0 = Pose()
            p1 = Pose()
            p0.position = (self.target_position[0], self.target_position[1], self.target_position[2])
            p1.position = (state.frames["wrist_3_link"].world_transform.translation.x, state.frames["wrist_3_link"].world_transform.translation.y, state.frames["wrist_3_link"].world_transform.translation.z)
            p0.orientation = self.goals["orientationMatch"]
            p1.orientation = state.frames["wrist_3_link"].world_transform.rotation
            poseWasReached = poseReached(p0, p1)
            joint_state_copy = state.joints.copy()
            # joint_state_copy["time"] = time
            print("p0 position: ", p0.position)
            print("p1 position: ", p1.position)
            
            nested_joint_state_copy = {
                "joints": joint_state_copy,
                "time": time
            }
            state_with_time_copy = {
                "state": state,
                "time": time 
            }
            time += INTERVAL_TIME
            # print("State: \n", state)
            # save the state to robot_states list
            states.append(nested_joint_state_copy)
            self.trajectory.append(nested_joint_state_copy)
            self.robot_states.append(self.state2tfs(state))
            self.states_with_time.append(state_with_time_copy)
            ########################## UR3
            # Define the desired order
            desired_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            #print("state.joints? ", state.joints)
            values = [state.joints["shoulder_pan_joint"], state.joints["shoulder_lift_joint"], state.joints["elbow_joint"],
                    state.joints["wrist_1_joint"], state.joints["wrist_2_joint"], state.joints["wrist_3_joint"]]
            # Sort the keys of state.joints based on the desired order
            #sorted_keys = sorted(state.joints, key=lambda x: desired_order.index(x))
            
            ###### Extract the values corresponding to the sorted keys
            #values = [state.joints[k] for k in sorted_keys]
            # Convert the values to a numpy array
            values_array = np.array(values)
            joint_states.append(values_array)
            
            if poseWasReached:
                # print("State reached: ", state)
                self.poseReached = True
                break
            i += 1

            if i % 200 == 0:
                print("breakpoint")

        print("Move Total iteration: ", i+1)
        if(self.poseReached):
            print("Executing trajectory was successful")
        else:
            print("Executing trajectory was unsuccessful, could not reach the desired position") 
        # Concatenate all arrays into one array
        self.joint_states = np.vstack(joint_states)
        # print(self.solver.links)
        print("list of joint states: ", self.joint_states)
        print(len(self.joint_states[0]))
        state_df = pd.DataFrame(self.joint_states)
        state_df.to_csv("state_0904_6pm.csv", header = False, index = False)
        
    def move_down(self, height=0.06):
        # Initialize the time variable
        time = 0.0  
        # Initialize the poseReached variable
        self.poseReached = False
        # if not moving to the first waypoint
        if(self.trajectory != []):     
            # update weights and goals
            self.objectives["jointMatch"] = JointMatchObjective(name="MyJointMatchObjective", joint = "robotiq_85_left_knuckle_joint", weight=10.0)
            self.objectives["positionMatch"] = PositionMatchObjective(name="MyPositionMatchObjective", link="gripper_offset", weight=30.0)
            # initialize updated solver
            self.solver = Solver(
            urdf=self.xml_string, # Full urdf as a string
            objectives=self.objectives,
            initial_state = State(origin=Transform(Translation(x=0.0, y=-0.15, z=0.76), Rotation(x=0.0, y=0.0, z=1.0, w=0.0)), joints=self.trajectory[-1]["joints"]),
            root_bounds=self.root_bounds,
            shapes = self.shapes
            )
            # update time variable
            time = self.trajectory[-1]["time"] + INTERVAL_TIME
        
        # List to collect the states
        joint_states = []
        robot_states = []
        states = []
        joint_state_copy = {}

        self.target_position = (self.target_position[0], self.target_position[1], self.target_position[2] - height)
        self.goals["positionMatch"] = Translation(self.target_position[0], self.target_position[1], self.target_position[2])
        max_iterations = 100
        i = 0

        while i < max_iterations:
            state = self.solver.solve(self.goals, weights = {},time = 0.0)
            p0 = Pose()
            p1 = Pose()
            p0.position = (self.target_position[0], self.target_position[1], self.target_position[2])
            p1.position = (state.frames["gripper_offset"].world_transform.translation.x, state.frames["gripper_offset"].world_transform.translation.y, state.frames["gripper_offset"].world_transform.translation.z)
            p0.orientation = self.goals["orientationMatch"]
            p1.orientation = state.frames["gripper_offset"].world_transform.rotation
            poseWasReached = poseReached(p0, p1)
            joint_state_copy = state.joints.copy()
            # joint_state_copy["time"] = time
            
            nested_joint_state_copy = {
                "joints": joint_state_copy,
                "time": time
            }
            state_with_time_copy = {
                "state": state,
                "time": time
            }
            time += INTERVAL_TIME
            # print("State: \n", state)
            # save the state to robot_states list
            states.append(nested_joint_state_copy)
            self.trajectory.append(nested_joint_state_copy)
            self.robot_states.append(self.state2tfs(state))
            self.states_with_time.append(state_with_time_copy)
            ########################## UR3
            # Define the desired order
            desired_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'robotiq_85_base_joint', 'robotiq_85_left_knuckle_joint', 'robotiq_85_right_knuckle_joint', 'robotiq_85_left_finger_joint', 'robotiq_85_right_finger_joint', 'robotiq_85_left_inner_knuckle_joint', 'robotiq_85_right_inner_knuckle_joint', 'robotiq_85_left_finger_tip_joint', 'robotiq_85_right_finger_tip_joint']

            # Sort the keys of state.joints based on the desired order
            sorted_keys = sorted(state.joints, key=lambda x: desired_order.index(x))
            
            ###### Extract the values corresponding to the sorted keys
            values = [state.joints[k] for k in sorted_keys]
            # Convert the values to a numpy array
            values_array = np.array(values)
            joint_states.append(values_array)
            
            if poseWasReached:
                # print("State reached: ", state)
                self.poseReached = True
                break
            i += 1

        print("Move Total iteration: ", i+1)
        if(self.poseReached):
            print("Executing trajectory was successful")
        else:
            print("Executing trajectory was unsuccessful, could not reach the desired position") 
        # Concatenate all arrays into one array
        self.joint_states = np.vstack(joint_states)
        # print(self.solver.links)
        # print("list of joint states: ", self.joint_states)  
        
    def move_up(self, height=0.1):
        # Initialize the time variable
        time = 0.0  
        # Initialize the poseReached variable
        self.poseReached = False
        # if not moving to the first waypoint
        if(self.trajectory != []):     
            # update weights and goals
            self.objectives["jointMatch"] = JointMatchObjective(name="MyJointMatchObjective", joint = "robotiq_85_left_knuckle_joint", weight=10.0)
            self.objectives["positionMatch"] = PositionMatchObjective(name="MyPositionMatchObjective", link="gripper_offset", weight=30.0)
            # initialize updated solver
            self.solver = Solver(
            urdf=self.xml_string, # Full urdf as a string
            objectives=self.objectives,
            initial_state = State(origin=Transform(Translation(x=0.0, y=-0.15, z=0.76), Rotation(x=0.0, y=0.0, z=1.0, w=0.0)), joints=self.trajectory[-1]["joints"]),
            root_bounds=self.root_bounds,
            shapes = self.shapes
            )
            # update time variable
            time = self.trajectory[-1]["time"] + INTERVAL_TIME
        
        # List to collect the states
        joint_states = []
        robot_states = []
        states = []
        joint_state_copy = {}

        self.target_position = (self.target_position[0], self.target_position[1], self.target_position[2] + height)
        self.goals["positionMatch"] = Translation(self.target_position[0], self.target_position[1], self.target_position[2])
        max_iterations = 300
        i = 0

        while i < max_iterations:
            state = self.solver.solve(self.goals, weights = {},time = 0.0)
            p0 = Pose()
            p1 = Pose()
            p0.position = (self.target_position[0], self.target_position[1], self.target_position[2])
            p1.position = (state.frames["gripper_offset"].world_transform.translation.x, state.frames["gripper_offset"].world_transform.translation.y, state.frames["gripper_offset"].world_transform.translation.z)
            p0.orientation = self.goals["orientationMatch"]
            p1.orientation = state.frames["gripper_offset"].world_transform.rotation
            poseWasReached = poseReached(p0, p1)
            joint_state_copy = state.joints.copy()
            # joint_state_copy["time"] = time
            
            nested_joint_state_copy = {
                "joints": joint_state_copy,
                "time": time
            }
            state_with_time_copy = {
                "state": state,
                "time": time
            }
            time += INTERVAL_TIME
            # print("State: \n", state)
            # save the state to robot_states list
            states.append(nested_joint_state_copy)
            self.trajectory.append(nested_joint_state_copy)
            self.robot_states.append(self.state2tfs(state))
            self.states_with_time.append(state_with_time_copy)
            ########################## UR3
            # Define the desired order
            desired_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'robotiq_85_base_joint', 'robotiq_85_left_knuckle_joint', 'robotiq_85_right_knuckle_joint', 'robotiq_85_left_finger_joint', 'robotiq_85_right_finger_joint', 'robotiq_85_left_inner_knuckle_joint', 'robotiq_85_right_inner_knuckle_joint', 'robotiq_85_left_finger_tip_joint', 'robotiq_85_right_finger_tip_joint']

            # Sort the keys of state.joints based on the desired order
            sorted_keys = sorted(state.joints, key=lambda x: desired_order.index(x))
            
            ###### Extract the values corresponding to the sorted keys
            values = [state.joints[k] for k in sorted_keys]
            # Convert the values to a numpy array
            values_array = np.array(values)
            joint_states.append(values_array)
            
            if poseWasReached:
                # print("State reached: ", state)
                self.poseReached = True
                break
            i += 1

        print("Move Total iteration: ", i+1)
        if(self.poseReached):
            print("Executing trajectory was successful")
        else:
            print("Executing trajectory was unsuccessful, could not reach the desired position") 
        # Concatenate all arrays into one array
        self.joint_states = np.vstack(joint_states)
        # print(self.solver.links)
        # print("list of joint states: ", self.joint_states)        
    
    def execute_trajectory(self):
        # Define the CSV file name
        csv_file = "joint_states.csv"
        json_file = "./example_state2tfs.json"
        states_json_file = "./states.json"
        states_with_time_json_file = "./states_with_time.json"
        # writing to json file to evaluate critic_joint_speed.py
        with open(states_json_file, 'w') as file:
            json.dump(self.trajectory, file, indent=4) 
        
        # writing to json file so frontend can run simulation of it
        with open(json_file, 'w') as file:
            json.dump(self.robot_states, file, indent=4)
            
        # writing to the CSV file so the pyhsiical robot can execute the trajectory if needed
        with open(csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)   
            writer.writerows(self.joint_states) 

        ## writing critic output to txt file
        output_content = []
        # printing result to terminal of evaluating critic_end_effector_pose.py
        # print(critic_end_effector_pose(self.states_with_time, gripperBaseID="robotiq_85_base_link", attachmentPointID="flange"))
        output_content.append(critic_end_effector_pose(self.states_with_time, gripperBaseID="robotiq_85_base_link", attachmentPointID="flange"))
        # printing result to terminal of evaluating critic_space_usage.py
        # print(critic_space_usage(self.states_with_time, robotWorkZoneVolume=0.523))
        output_content.append(critic_space_usage(self.states_with_time, robotWorkZoneVolume=0.523))
        # printing result to terminal of evaluating critic_pinch_points.py
        # print(critic_pinch_points(self.states_with_time))
        output_content.append(critic_pinch_points(self.states_with_time))
        # printing result to terminal of evaluating critic_joint_speed.py
        # print(critic_joint_speed(self.trajectory, "./Lib/ur3.xml"))
        output_content.append(critic_joint_speed(self.trajectory, "./Lib/ur3.xml"))
        with open("critics.txt", "w") as file:
            for line in output_content:
                print(line, file=file)
    
    def run_critics(self):
        # printing result to terminal of evaluating critic_end_effector_pose.py
        print(critic_end_effector_pose(self.states_with_time, gripperBaseID="robotiq_85_base_link", attachmentPointID="flange"))
        # printing result to terminal of evaluating critic_space_usage.py
        print(critic_space_usage(self.states_with_time, robotWorkZoneVolume=0.523))
        # printing result to terminal of evaluating critic_pinch_points.py
        print(critic_pinch_points(self.states_with_time))
        # printing result to terminal of evaluating critic_joint_speed.py
        print(critic_joint_speed(self.trajectory, "./Lib/ur3.xml"))    
    
    def avoid_collision(self, obj_name):        
        with open('./Lib/environment.json', 'r') as file:
            data = json.load(file)
        offset = 0.08    
        for obj in data:
            if obj_name in obj:
                # print(obj["red_block"]["scale"]["x"])
                item = obj[obj_name]
                box = BoxShape(name=obj_name,frame="world",physical=True,x=item["scale"]["x"], y=item["scale"]["y"], z=item["scale"]["z"], translation = Translation(x=item["position"]["x"], y =item["position"]["y"], z=item["position"]["z"]+item["scale"]["z"]),
                rotation = Rotation(x=item["rotation"]["x"], y=item["rotation"]["y"], z=item["rotation"]["z"], w=item["rotation"]["w"]))
                self.shapes.append(box)   
    
    def reduce_speed(self):
        global INTERVAL_TIME
        INTERVAL_TIME += 45
    
    def increase_speed(self):
        global INTERVAL_TIME
        INTERVAL_TIME -= 15    
            
    def close_gripper(self):
        # Initialize the time variable
        time = 0.0  
        # Initialize the target value
        target = 0.5
        # Initialize the jointReached variable
        self.jointReached = False
        # update weights and goals
        self.objectives["jointMatch"] = JointMatchObjective(name="MyJointMatchObjective", joint = "robotiq_85_left_knuckle_joint", weight=20.0)
        self.objectives["positionMatch"] = PositionMatchObjective(name="MyPositionMatchObjective", link="gripper_offset", weight=10.0)
        # initialize updated solver
        self.solver = Solver(
        urdf=self.xml_string, # Full urdf as a string
        objectives=self.objectives,
        initial_state = State(origin=Transform(Translation(x=0.0, y=-0.15, z=0.76), Rotation(x=0.0, y=0.0, z=1.0, w=0.0)), joints=self.trajectory[-1]["joints"]),
        root_bounds=self.root_bounds,
        shapes = self.shapes
        )
        # update time variable
        time = self.trajectory[-1]["time"] + INTERVAL_TIME
        
        # List to collect the states
        joint_states = []
        robot_states = []
        states = []
        joint_state_copy = {}
        self.goals["jointMatch"] = target
        max_iterations = 100
        i = 0
        while i < max_iterations:
            state = self.solver.solve(self.goals, weights = {},time = 0.0)
            jointWasReached = jointReached(state.joints["robotiq_85_left_knuckle_joint"], target, 0.1)
            joint_state_copy = state.joints.copy()
            
            nested_joint_state_copy = {
                "joints": joint_state_copy,
                "time": time
            }
            state_with_time_copy = {
                "state": state,
                "time": time
            }
            time += INTERVAL_TIME 
            # print("State: \n", state)
            # save the state to robot_states list
            states.append(nested_joint_state_copy)
            self.trajectory.append(nested_joint_state_copy)
            self.robot_states.append(self.state2tfs(state))
            self.states_with_time.append(state_with_time_copy)
            ########################## UR3
            # Define the desired order
            desired_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'robotiq_85_base_joint', 'robotiq_85_left_knuckle_joint', 'robotiq_85_right_knuckle_joint', 'robotiq_85_left_finger_joint', 'robotiq_85_right_finger_joint', 'robotiq_85_left_inner_knuckle_joint', 'robotiq_85_right_inner_knuckle_joint', 'robotiq_85_left_finger_tip_joint', 'robotiq_85_right_finger_tip_joint']

            # Sort the keys of state.joints based on the desired order
            sorted_keys = sorted(state.joints, key=lambda x: desired_order.index(x))
            
            ###### Extract the values corresponding to the sorted keys
            values = [state.joints[k] for k in sorted_keys]
            # Convert the values to a numpy array
            values_array = np.array(values)
            joint_states.append(values_array)
            
            if jointWasReached:
                self.jointReached = True
                break
            i += 1
        
        print("Gripper Total iteration: ", i)
        if(self.jointReached):
            print("Executing trajectory was successful")
        else:
            print("Executing trajectory was unsuccessful, could not reach the desired joint value")
        # Concatenate all arrays into one array
        self.joint_states = np.vstack(joint_states)        
    
    def open_gripper(self):
        # Initialize the time variable
        time = 0.0  
        # Initialize the jointReached variable
        self.jointReached = False
        # Initialize the target value
        target = 0.0
        # update weights and goals
        self.objectives["jointMatch"] = JointMatchObjective(name="MyJointMatchObjective", joint = "robotiq_85_left_knuckle_joint", weight=20.0)
        self.objectives["positionMatch"] = PositionMatchObjective(name="MyPositionMatchObjective", link="gripper_offset", weight=0.0)
        # initialize updated solver
        self.solver = Solver(
        urdf=self.xml_string, # Full urdf as a string
        objectives=self.objectives,
        initial_state = State(origin=Transform(Translation(x=0.0, y=-0.15, z=0.76), Rotation(x=0.0, y=0.0, z=1.0, w=0.0)), joints=self.trajectory[-1]["joints"]),
        root_bounds=self.root_bounds,
        shapes = self.shapes
        )
        # update time variable
        time = self.trajectory[-1]["time"] + INTERVAL_TIME
        
        # List to collect the states
        joint_states = []
        robot_states = []
        states = []
        joint_state_copy = {}
        self.goals["jointMatch"] = target
        max_iterations = 100
        i = 0
        while i < max_iterations:
            state = self.solver.solve(self.goals, weights = {},time = 0.0)
            jointWasReached = jointReached(state.joints["robotiq_85_left_knuckle_joint"], target, 0.1)
            joint_state_copy = state.joints.copy()
            
            nested_joint_state_copy = {
                "joints": joint_state_copy,
                "time": time
            }
            state_with_time_copy = {
                "state": state,
                "time": time
            }
            time += INTERVAL_TIME
            # print("State: \n", state)
            # save the state to robot_states list
            states.append(nested_joint_state_copy)
            self.trajectory.append(nested_joint_state_copy)
            self.robot_states.append(self.state2tfs(state))
            self.states_with_time.append(state_with_time_copy)
            ########################## UR3
            # Define the desired order
            desired_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'robotiq_85_base_joint', 'robotiq_85_left_knuckle_joint', 'robotiq_85_right_knuckle_joint', 'robotiq_85_left_finger_joint', 'robotiq_85_right_finger_joint', 'robotiq_85_left_inner_knuckle_joint', 'robotiq_85_right_inner_knuckle_joint', 'robotiq_85_left_finger_tip_joint', 'robotiq_85_right_finger_tip_joint']

            # Sort the keys of state.joints based on the desired order
            sorted_keys = sorted(state.joints, key=lambda x: desired_order.index(x))
            
            ###### Extract the values corresponding to the sorted keys
            values = [state.joints[k] for k in sorted_keys]
            # Convert the values to a numpy array
            values_array = np.array(values)
            joint_states.append(values_array)
            
            if jointWasReached:
                self.jointReached = True
                break
            i += 1
        
        print("Gripper Total iteration: ", i)
        if(self.jointReached):
            print("Executing trajectory was successful")
        else:
            print("Executing trajectory was unsuccessful, could not reach the desired joint value")
        # Concatenate all arrays into one array
        self.joint_states = np.vstack(joint_states) 
        
    def rotate_gripper(self):
        # Initialize the time variable
        time = 0.0  
        # Initialize the target value, rotating the gripper 90 degrees around z axis
        target = Rotation(x=0, y=0, z=0.707, w=0.707) 
        # Initialize the jointReached variable
        self.orientationReached = False
        # update weights and goals
        self.objectives["jointMatch"] = JointMatchObjective(name="MyJointMatchObjective", joint = "robotiq_85_left_knuckle_joint", weight=10.0)
        self.objectives["positionMatch"] = PositionMatchObjective(name="MyPositionMatchObjective", link="gripper_offset", weight=10.0)
        self.objectives["orientationMatch"] = OrientationMatchObjective(name="MyOrientationMatchObjective", link="gripper_offset", weight=20.0)
        # initialize updated solver
        self.solver = Solver(
        urdf=self.xml_string, # Full urdf as a string
        objectives=self.objectives,
        initial_state = State(origin=Transform(Translation(x=0.0, y=-0.15, z=0.76), Rotation(x=0.0, y=0.0, z=1.0, w=0.0)), joints=self.trajectory[-1]["joints"]),
        root_bounds=self.root_bounds,
        shapes = self.shapes
        )
        # update time variable
        time = self.trajectory[-1]["time"] + INTERVAL_TIME
        
        # List to collect the states
        joint_states = []
        robot_states = []
        states = []
        joint_state_copy = {}
        q = self.multiply_quaternions(self.goals["orientationMatch"], target)
        self.goals["orientationMatch"] = Rotation(x=q[0], y=q[1], z=q[2], w=q[3])
        max_iterations = 100
        i = 0
        while i < max_iterations:
            state = self.solver.solve(self.goals, weights = {},time = 0.0)
            orientationWasReached = orientationReached(self.goals["orientationMatch"], state.frames["gripper_offset"].world_transform.rotation)
            joint_state_copy = state.joints.copy()
            
            nested_joint_state_copy = {
                "joints": joint_state_copy,
                "time": time
            }
            state_with_time_copy = {
                "state": state,
                "time": time
            }
            time += INTERVAL_TIME
            # print("State: \n", state)
            # save the state to robot_states list
            states.append(nested_joint_state_copy)
            self.trajectory.append(nested_joint_state_copy)
            self.robot_states.append(self.state2tfs(state))
            self.states_with_time.append(state_with_time_copy)
            ########################## UR3
            # Define the desired order
            desired_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'robotiq_85_base_joint', 'robotiq_85_left_knuckle_joint', 'robotiq_85_right_knuckle_joint', 'robotiq_85_left_finger_joint', 'robotiq_85_right_finger_joint', 'robotiq_85_left_inner_knuckle_joint', 'robotiq_85_right_inner_knuckle_joint', 'robotiq_85_left_finger_tip_joint', 'robotiq_85_right_finger_tip_joint']

            # Sort the keys of state.joints based on the desired order
            sorted_keys = sorted(state.joints, key=lambda x: desired_order.index(x))
            
            ###### Extract the values corresponding to the sorted keys
            values = [state.joints[k] for k in sorted_keys]
            # Convert the values to a numpy array
            values_array = np.array(values)
            joint_states.append(values_array)
            
            if orientationWasReached:
                self.orientationReached = True
                break
            i += 1
        
        print("Gripper Rotation Total iteration: ", i)
        if(self.orientationReached):
            print("Executing rotation was successful")
        else:
            print("Executing rotation was unsuccessful, could not reach the desired rotation value")
        # Concatenate all arrays into one array
        self.joint_states = np.vstack(joint_states)             
             
            
    def state2tfs(self, state):
        tfs = {}
        # Uncomment the following line to print the state's 'proximity' if needed for debugging
        # print(state['proximity'])
        for key, value in state.frames.items():
            world = value.world_transform
            translation = world.translation
            rotation = world.rotation
            
            tfs[key] = {
                'frame': 'world',
                'position': {
                    'x': translation.x,
                    'y': translation.y,
                    'z': translation.z,
                },
                'rotation': {
                    'w': rotation.w,
                    'x': rotation.x,
                    'y': rotation.y,
                    'z': rotation.z,
                },
                'scale': {
                    'x': 1,
                    'y': 1,
                    'z': 1,
                },
            }
               
    
        return tfs    
  
    def control_robot():
        print("starting program")

        # Define constants and parameters
        HOST = "169.254.76.240"  # The remote host (the robot)
        PORT = 30002  # same port with server
        time_between = 0.09
        gain = 100
        lookahead_time = 0.1
        a = 1
        v = 0
        middle_slowdown = 15
        wait = 0.5
        end_pause = 15
        time_sleep = 0.045

        # Connect to robot
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("connecting...")
        s.connect((HOST, PORT))
        time.sleep(0.5)
        print("connected")

        # Move to starting position
        with open("joint_states.csv", "r") as my_file:
            print("open csv file")
            file_reader = reader(my_file)
            for i in file_reader:
                q = list(map(float, i))  
                command = (
                    "servoj([{0},{1},{2},{3},{4},{5}],t=3)".format(
                        str(q[0]), str(q[1]), str(q[2]), str(q[3]), str(q[4]), str(q[5])
                    ) 
                    + "\n"
                )
                s.send(command.encode("utf8"))
                time.sleep(3)
                break

        # Reading in file and sending coordinates to robot
        with open("joint_states.csv", "r") as my_file:
            file_reader = reader(my_file)
            for i in file_reader:
                if i[0] == "PAUSE":
                    for x in range(middle_slowdown):
                        command = (
                            "servoj([{0},{1},{ 2},{3},{4},{5}], a={6}, v={7}, t={8}, lookahead_time={9}, gain={10})".format(
                                str(q[0]), str(q[1]), str(q[2]), str(q[3]), str(q[4]), str(q[5]), str(a), str(v), str(time_between), str(lookahead_time), str(gain)
                            ) 
                            + "\n"
                        ) 
                        s.send(command.encode("utf8"))
                        time.sleep(time_sleep)
                    time.sleep(wait)
                    continue

                q = list(map(float, i))  
                command = (
                    "servoj([{0},{1},{2},{3},{4},{5}], a={6}, v={7}, t={8}, lookahead_time={9}, gain={10})".format(
                        str(q[0]), str(q[1]), str(q[2]), str(q[3]), str(q[4]), str(q[5]), str(a), str(v), str(time_between), str(lookahead_time), str(gain)
                    ) 
                    + "\n"
                )   
                s.send(command.encode("utf8"))
                time.sleep(time_sleep)

        for x in range(end_pause):
            command = (
                "servoj([{0},{1},{2},{3},{4},{5}], a={6}, v={7}, t={8}, lookahead_time={9}, gain={10})".format(
                    str(q[0]), str(q[1]), str(q[2]), str(q[3]), str(q[4]), str(q[5]), str(a), str(v), str(time_between), str(lookahead_time), str(gain)
                ) 
                + "\n"
            ) 
            s.send(command.encode("utf8"))
            time.sleep(time_sleep)

        # Close connection
        s.close()    
    
    def multiply_quaternions(self, q1, q2):
        # Extract the components of the quaternions
        x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
        x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w
        
        # Calculate the components of the resulting quaternion
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        
        return [x, y, z, w]   
        
# For testing purposes
if __name__ == "__main__":
    my_instance = FunctionLib()  # This will create instane of FunctionLib class and run the __init__ method
    # my_instance.get_trajectory(1.0, 1.0, 1.0) # This will call the get_trajectory method
    # my_instance.execute_trajectory() # This will call the execute_trajectory method
    # my_instance.control_robot() # This will call the control_robot method
    # my_instance.state2tfs(my_instance.initial_state)
    move_to = my_instance.move_to(0.1, 0, 0.64)
# def open_gripper():
#     # do somethjing
#     return  

# def move_to(x, y, z):
#     # do somethjing
#     return   

# def close_gripper(obj_name):
#     # do somethjing
#     return 