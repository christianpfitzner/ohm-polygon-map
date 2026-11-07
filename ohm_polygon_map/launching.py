"""What `launch/box.launch.py` starts: the simulator in the hall `box`, one robot that stands, RViz.

The simulator's own `lab.launch.py` (repository `ir-mecanum-lab`) is included with `worlds/box.txt` and
`config/box.json`, and with its own RViz switched off: this lab brings an RViz config of its own that also
shows the topics of the students' node (`rviz/polygon_map.rviz`). The students' node is not started here;
they run it themselves in a second terminal.
"""
import os
import shutil

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, \
    OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from . import paths

ARGS = [
    ("robot", "muster", "robot name"),
    ("start", "slanted", "start pose: slanted (20 deg, the lab) | straight (0 deg, extra task Z.1)"),
    ("rviz", "auto", "RViz 2 with the lab's view: auto | true | false"),
    ("headless", "false", "without the window (SDL dummy driver)"),
    ("seconds", "", "end after N s of simulation time (empty = until Ctrl-C)"),
]
NODE = "ros2 run ohm_polygon_map polygon_map_node.py --ros-args -p stage:=1"


def rviz_config(robot: str) -> str:
    """Fill the `{robot}` placeholder of the lab's RViz template; returns the generated file."""
    with open(paths.lab_file("rviz", "polygon_map.rviz"), encoding="utf-8") as handle:
        text = handle.read()
    target = f"/tmp/ohm_polygon_map_{robot}.rviz"
    with open(target, "w", encoding="utf-8") as handle:
        handle.write(text.replace("{robot}", robot))
    return target


def _start(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)   # noqa: E731
    simulator = paths.simulator_root()
    configs = {"slanted": "box.json", "straight": "box_straight.json"}
    if arg("start") not in configs:
        raise ValueError(f"start:={arg('start')} — choose one of: {', '.join(configs)}")
    forwarded = {"world": paths.lab_file("worlds", "box.txt"),
                 "config": paths.lab_file("config", configs[arg("start")]),
                 "robot": arg("robot"), "rviz": "false", "headless": arg("headless")}
    if arg("seconds"):
        forwarded["seconds"] = arg("seconds")
    actions = [LogInfo(msg=f"[polygon map] robot '{arg('robot')}' stands in the hall 'box' "
                           f"(simulator {simulator}); start your node in a second terminal: {NODE}"),
               IncludeLaunchDescription(
                   PythonLaunchDescriptionSource(os.path.join(simulator, "launch", "lab.launch.py")),
                   launch_arguments=forwarded.items())]
    wanted = arg("rviz").lower()
    if wanted == "true" or (wanted == "auto" and shutil.which("rviz2")):
        actions.append(ExecuteProcess(cmd=["rviz2", "--display-config", rviz_config(arg("robot")),
                                           "--ros-args", "-p", "use_sim_time:=true"],
                                      name="rviz2", output="screen"))
    elif wanted == "auto":
        actions.append(LogInfo(msg="[polygon map] rviz2 is not installed: "
                                   "sudo apt install ros-$ROS_DISTRO-rviz2"))
    return actions


def describe() -> LaunchDescription:
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default, description=text)
                              for name, default, text in ARGS]
                             + [OpaqueFunction(function=_start)])
