from lively import Solver, State, Translation, Rotation, Transform, PositionMatchObjective, OrientationMatchObjective, ScalarRange
from lxml import etree

from Pose import Pose
from pose_reached import poseReached

import numpy as np 

INTERVAL_TIME = 0.1

xml_file = "ur3e.xml"
tree = etree.parse(xml_file)
xml_string = etree.tostring(tree).decode()
initial_state = State(origin = Transform(Translation(x = 0.0, y = 0.0, z = 0.59), Rotation(x = 0.0, y = 0.0, z = 1.0, w = 0.0)), joints = 
				{'shoulder_pan_joint': 0, 
				'shoulder_lift_joint': 0,
				'elbow_joint': 0,
				'wrist_1_joint': 0,
				'wrist_2_joint': 0,
				'wrist_3_joint': 1})
objectives = {"positionMatch": PositionMatchObjective(name = "popo", weight = 50, link = 'wrist_3_joint'),
			"orientationMatch": OrientationMatchObjective(name = "oror", weight = 30.3, link = 'wrist_3_joint')}
root_bounds = [ScalarRange(value = 0.0, delta = 0.0), ScalarRange(value = 0.0, delta = 0.0), ScalarRange(value = 0.59, delta = 0.0),
			ScalarRange(value = 0.0, delta = 0.0), ScalarRange(value = 0.0, delta = 0.0), ScalarRange(value = 0.0, delta = 0.0)]

solver = Solver(
		urdf = xml_string,
		objectives = objectives,
		root_bounds = root_bounds,
		initial_state = initial_state
		)

trajectory = []
robot_states = []
joint_states = []
states_with_time = []
states = []
joint_state_copy = {}

goals = {}
goals["positionMatch"] = Translation(x = 0.0, y = 0.0, z = 1.59)
goals["orientationMatch"] = Rotation(x = 0.0, y = 1.0, z = 0.0, w = 0.0)

i= 0; max_iterations = 500
poseWasReached = False

time = 0.0
while (i < max_iterations):
	state = solver.solve(goals, weights = {}, time = 0.0)

	p0 = Pose(); p1 = Pose()
	p0.position= (0, 0, 1.59)
	p1.position = (state.frames["wrist_3_link"].world_transform.translation.x, 
				state.frames["wrist_3_link"].world_transform.translation.y,
				state.frames["wrist_3_link"].world_transform.translation.z)
	p0.orientation = goals["orientationMatch"]
	p1.orientation = state.frames["wrist_3_link"].world_transform.rotation 

	poseWasReached = poseReached(p0, p1)

	joint_state_copy = state.joints.copy()
	nested_joint_state_copy = {"joints": joint_state_copy, "time": time}
	state_with_time_copy = {"state": state, "time": time}
	time += INTERVAL_TIME

	states.append(nested_joint_state_copy)
	trajectory.append(nested_joint_state_copy)
	states_with_time.append(state_with_time_copy)

	desired_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
	sorted_keys = sorted(state.joints, key = lambda x: desired_order.index(x))
	values = [state.joints[k] for k in sorted_keys]
	values_array = np.array(values)
	joint_states.append(values_array)

	if poseWasReached:
		print("success!")
		break
	i += 1

	# if (i % 10 == 0):
	# 	print(poseWasReached)
	# 	print(p1.position)
	print(poseWasReached)
	print(p1.position)

print(joint_states[0])
print("***")
print(joint_states[-1])