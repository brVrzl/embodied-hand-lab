from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "python3",
                    "-m",
                    "data_recorder.cli",
                    "--task",
                    "pick_and_place",
                    "--instruction",
                    "pick the cube and place it into the tray",
                ],
                output="screen",
            )
        ]
    )

