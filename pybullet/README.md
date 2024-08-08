# UR3e_simulation
URDF Test for UR3e robot arm using pybullet. URDFs and Meshes are from [here](https://github.com/Daniella1/urdf_files_dataset/blob/main/urdf_files/ros-industrial/xacro_generated/universal_robots/ur_description/urdf/ur3e.urdf).

## Setup

You need to install python3. Once that is done, set up your virtual environment (venv):

```bash
python3 -m venv venv
```

Now activate your venv:

```bash
# Mac/Linux:
source venv/bin/activate

# Windows:
venv\Scripts\activate
```

Then run these following commands in this directory to install all dependencies:

```bash
pip install -r requirements.txt
```

## Running

To run this, execute the following command and a pybullet simulation window should pop up:
```bash
python3 main.py
```
