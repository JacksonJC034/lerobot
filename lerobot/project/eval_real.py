
import time
from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.cameras.opencv import OpenCVCamera
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.scripts.control_robot import busy_wait
import torch

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
        "phone": OpenCVCamera(4, fps=30, width=640, height=480),
        "webcam": OpenCVCamera(2, fps=30, width=640, height=480)
    },
)
robot.connect()

observation, action = robot.teleop_step(record_data=True)


"""
Run the trained policy on the koch arm
"""
inference_time_s = 60
fps = 30
device = "cuda"  # TODO: On Mac, use "mps" or "cpu"
ckpt_path = "underctrl/lerobot_data_collection"
policy = ACTPolicy.from_pretrained(ckpt_path)
policy.to(device)

for _ in range(inference_time_s * fps):
    start_time = time.perf_counter()

    # Read the follower state and access the frames from the cameras
    observation = robot.capture_observation()

    # Convert to pytorch format: channel first and float32 in [0,1]
    # with batch dimension
    for name in observation:
        if "image" in name:
            observation[name] = observation[name].type(torch.float32) / 255
            observation[name] = observation[name].permute(2, 0, 1).contiguous()
        observation[name] = observation[name].unsqueeze(0)
        observation[name] = observation[name].to(device)

    # Compute the next action with the policy
    # based on the current observation
    action = policy.select_action(observation)
    # Remove batch dimension
    action = action.squeeze(0)
    # Move to cpu, if not already the case
    action = action.to("cpu")
    # Order the robot to move
    robot.send_action(action)

    dt_s = time.perf_counter() - start_time
    busy_wait(1 / fps - dt_s)
    
# robot.disconnect()