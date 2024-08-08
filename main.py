import pybullet as p
import time
import pybullet_data

physics_client = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

# Load ground
plane_ID = p.loadURDF("plane.urdf")

# Load robot
start_pos = [0,0,0]
start_orientation = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF("robot.urdf",start_pos, start_orientation)


# Map joint names (from urdf) to index values
joints = {}  

for i in range(p.getNumJoints(robot)):
    joint_name = p.getJointInfo(robot, i)[1].decode('UTF-8') 
    joints[joint_name] = i


# Zoom in w/ camera
p.resetDebugVisualizerCamera(cameraDistance=1,
                             cameraYaw=50,
                             cameraPitch=-35,
                             cameraTargetPosition=[0, 0, 0])



# Initial Position
position = 0.0  

last_time = time.time()
INCREMENT_INTERVAL = 0.1  # Time in seconds to wait before next increment
POSITION_INCREMENT = 0.05  # Amount by which to increase the joint position

# Increment joint(s) by POSITION_INCREMENT every INCREMENT_INTERVAL
# Bounds of rotation limited by urdf
while True:

    current_time = time.time()

    # Check if it's time to update the joint position
    if current_time - last_time >= INCREMENT_INTERVAL:
        position += POSITION_INCREMENT

        # Increments position of wrist_1_joint joint
        p.setJointMotorControl2(robot, joints["wrist_1_joint"], p.POSITION_CONTROL, targetPosition=position)

        last_time = current_time

    p.stepSimulation()

    # Sleep to simulate real time (this can be adjusted or removed as needed)
    time.sleep(1/240)

p.disconnect()