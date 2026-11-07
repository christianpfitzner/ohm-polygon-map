"""Lab "Polygon map": the simulator in the hall `box`, the robot stands, RViz shows scan and map.

    ros2 launch ohm_polygon_map box.launch.py
    ros2 run ohm_polygon_map polygon_map_node.py --ros-args -p stage:=1      # second terminal
    ros2 launch ohm_polygon_map box.launch.py start:=straight                 # extra task: heading 0
    ros2 launch launch/box.launch.py                                          # from the checkout

`--show-args` lists the arguments; ohm_polygon_map/launching.py does the work.
"""
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))   # checkout, even via a symlink
if HERE not in sys.path:
    sys.path.append(HERE)
from ohm_polygon_map.launching import describe                  # noqa: E402


def generate_launch_description():
    return describe()
