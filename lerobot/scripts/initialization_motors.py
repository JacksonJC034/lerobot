from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus

leader_port = "/dev/ttyACM1"
follower_port = "/dev/ttyACM0"

leader_arm = DynamixelMotorsBus(
    port=leader_port,
    motors={
        # name: (index, model)
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
        # name: (index, model)
        "shoulder_pan": (0, "xl430-w250"),
        "shoulder_lift": (1, "xl430-w250"),
        "elbow_flex": (2, "xl330-m288"),
        "wrist_flex": (3, "xl330-m288"),
        "wrist_roll": (4, "xl330-m288"),
        "gripper": (5, "xl330-m288"),
    },
)