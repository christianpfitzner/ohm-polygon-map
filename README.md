# ohm_polygon_map

Lab *Polygon map* for the simulator [ir-mecanum-lab](https://github.com/christianpfitzner/ir-mecanum-lab):
four walls, four corners and one circle from a single laser scan, with least squares and RANSAC.
Handout: `docs/polygon_map.pdf` (`make` in `docs/`).

## Install

```bash
cd ~/ros2_ws/src
git clone git@github.com:christianpfitzner/ir-mecanum-lab.git
git clone git@github.com:christianpfitzner/ohm-polygon-map.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select mecanum_lab_interfaces mecanum_lab ohm_polygon_map
source install/setup.bash
```

## Run

| command | what |
|---|---|
| `ros2 launch ohm_polygon_map box.launch.py` | simulator in the hall `box`, the robot stands, RViz |
| `ros2 run ohm_polygon_map polygon_map_node.py --ros-args -p stage:=1` | the node, stages 0–5 |
| `ros2 launch ohm_polygon_map box.launch.py start:=straight` | extra task: heading 0, with `-p hesse:=true` |
| `python3 hint.py 1.3` | the next of three hints for `STUDENT 1.3`; without a block: the hints opened so far |

Your code: `student/line_fitting.py`, `student/circle_fitting.py`. Launch arguments: `start:=`, `robot:=`, `rviz:=`, `headless:=`, `seconds:=`.

## Parameters of the node

| parameter | default | |
|---|---|---|
| `stage` | 5 | 0 noise · 1 RANSAC · 2 least squares · 3 four lines · 4 corners · 5 circle |
| `tau` | 0 | inlier threshold in m, 0 = 3 σ from stage 0 |
| `w_line`, `w_circle`, `p` | 0.3, 0.5, 0.99 | RANSAC iterations k from these |
| `min_inliers`, `r_max`, `seed` | 20, 2.0, 1 | |
| `hesse` | false | extra task: stages 3 and 4 in Hesse normal form |

## Tests

```bash
python3 -m pytest test -q -k stage1
```
