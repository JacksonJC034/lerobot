
import time
from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.cameras.opencv import OpenCVCamera
from lerobot.scripts.control_robot import busy_wait


"""
Define the arms of korch robot
"""
leader_port = "/dev/ttyACM0"
follower_port = "/dev/ttyACM1"

leader_arm = DynamixelMotorsBus(
    port=leader_port,
    motors={
        "shoulder_pan": (0, "xl330-m077"),
        "shoulder_lift": (1, "xl330-m077"),
        "elbow_flex": (2, "xl330-m077"),
        "wrist_flex": (3, "xl330-m077"),
        "wrist_roll": (4, "xl330-m077"),
        "gripper": (5, "xl330-m077"),
    },
)

follower_arm = DynamixelMotorsBus(
    port=follower_port,
    motors={
        "shoulder_pan": (0, "xl430-w250"),
        "shoulder_lift": (1, "xl430-w250"),
        "elbow_flex": (2, "xl330-m288"),
        "wrist_flex": (3, "xl330-m288"),
        "wrist_roll": (4, "xl330-m288"),
        "gripper": (5, "xl330-m288"),
    },
)


"""
Instantiate the robot with camera
"""
robot = ManipulatorRobot(
    leader_arms={"main": leader_arm},
    follower_arms={"main": follower_arm},
    calibration_dir=".cache/calibration/koch",
    cameras={
        "iPhone": OpenCVCamera(4, fps=30, width=640, height=480),
        "webcam": OpenCVCamera(2, fps=30, width=640, height=480)
    },
)
robot.connect()

observation, action = robot.teleop_step(record_data=True)
print(observation["observation.images.iPhone"].shape)
print(observation["observation.images.iPhone"].min().item())
print(observation["observation.images.iPhone"].max().item())

# robot.disconnect()


"""
Record data
"""
record_time_s = 30
fps = 60

states = []
actions = []
for _ in range(record_time_s * fps):
    start_time = time.perf_counter()
    observation, action = robot.teleop_step(record_data=True)

    states.append(observation["observation.state"])
    actions.append(action["action"])

    dt_s = time.perf_counter() - start_time
    busy_wait(1 / fps - dt_s)

# Note that observation and action are available in RAM, but
# you could potentially store them on disk with pickle/hdf5 or
# our optimized format `LeRobotDataset`. More on this next.