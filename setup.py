"""The lab as a ROS 2 package: `colcon build --symlink-install`, then `ros2 launch ohm_polygon_map …`.

`student/` is installed into lib/: the node as an executable (`ros2 run ohm_polygon_map
polygon_map_node.py`), `line_fitting.py` and `circle_fitting.py` next to it. With --symlink-install they
are symlinks to the files a group edits.
"""
from glob import glob
import os

from setuptools import setup

PACKAGE = "ohm_polygon_map"

setup(
    name=PACKAGE,
    version="0.1.0",
    description="Lab 'Polygon map' for the mecanum-lab simulator",
    author="Praktikum Intelligente Robotik",
    license="MIT",
    python_requires=">=3.10",
    packages=[PACKAGE],
    data_files=[
        (os.path.join("share", "ament_index", "resource_index", "packages"), ["resource/" + PACKAGE]),
        (os.path.join("share", PACKAGE), ["package.xml"]),
        (os.path.join("share", PACKAGE, "launch"), glob("launch/*.py")),
        (os.path.join("share", PACKAGE, "config"), glob("config/*.json")),
        (os.path.join("share", PACKAGE, "worlds"), glob("worlds/*.txt")),
        (os.path.join("share", PACKAGE, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("lib", PACKAGE), glob("student/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
