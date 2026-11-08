"""Tests of the lab repository: the hall, the config and the launch file fit the simulator next door,
and the whole pipeline of the student files maps the room from one simulated scan.

    python3 -m pytest test -q          (the launch test needs ROS 2 sourced, the rest does not)
"""
import contextlib
import importlib.util
import io
import math
import os
import random
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "student"))
from ohm_polygon_map import paths                                 # noqa: E402

paths.use_simulator()
from mecanum_lab import sensors, worlds                           # noqa: E402
from mecanum_lab.types import load_config                         # noqa: E402

import circle_fitting as C                                        # noqa: E402
import line_fitting as F                                          # noqa: E402
from ohm_polygon_map import geometry as G                         # noqa: E402

ROOM = [(0.5, 0.5), (9.5, 0.5), (9.5, 6.5), (0.5, 6.5)]           # inner corners of worlds/box.txt


def load(rel):
    spec = importlib.util.spec_from_file_location(rel.replace("/", "_")[:-3], os.path.join(ROOT, rel))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def hall(config="box.json"):
    cfg = load_config(paths.lab_file("config", config))
    return worlds.apply_config(worlds.load_world(paths.lab_file("worlds", "box.txt"), cfg=cfg), cfg), cfg


def to_laser(pose, x, y):
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    return c * (x - pose.x) + s * (y - pose.y), -s * (x - pose.x) + c * (y - pose.y)


def test_the_hall_is_9_by_6_metres_inside_with_one_circle_and_a_slanted_start():
    world, _ = hall()
    assert world.name == "box" and world.size == (10.0, 7.0)
    assert len(world.circles) == 1
    assert math.degrees(world.spawn_pose(0).theta) == pytest.approx(20.0)


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_the_pipeline_maps_the_room_from_one_scan(seed):
    world, cfg = hall()
    pose = world.spawn_pose(0)
    lidar = sensors.Lidar(world, sensors.Noise(seed), cfg["lidar"])
    beam0 = [lidar.scan(pose).ranges[250] for _ in range(100)]
    tau = 3.0 * F.mean_and_std(beam0)[1]
    assert tau == pytest.approx(3.0 * cfg["lidar"]["sigma"], rel=0.25)

    scan = lidar.scan(pose)
    points = G.scan_to_points(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
                              scan.range_max)
    rng = random.Random(seed)
    lines, rest = F.find_lines(points, 4, tau, F.num_iterations(0.3, 2), 20, rng)
    corners = G.room_corners([line for line, _ in lines], F.intersect)
    assert len(corners) == 4
    for corner in (to_laser(pose, *c) for c in ROOM):            # the hidden corner too
        assert min(math.dist(corner, c) for c in corners) < 0.03
    assert sorted(G.polygon_sides(corners)) == pytest.approx([6.0, 6.0, 9.0, 9.0], abs=0.02)

    circle, inliers = C.ransac_circle(rest, tau, F.num_iterations(0.5, 3), rng, 2.0)
    x, y, r = world.circles[0].x, world.circles[0].y, world.circles[0].r
    assert math.dist(circle[:2], to_laser(pose, x, y)) < 0.05
    assert circle[2] == pytest.approx(r, abs=0.05)
    assert len(inliers) >= 15


def test_the_upper_right_corner_is_not_in_the_scan():
    world, cfg = hall()
    pose = world.spawn_pose(0)
    scan = sensors.Lidar(world, sensors.Noise(1), cfg["lidar"]).scan(pose)
    points = G.scan_to_points(scan.ranges, 0.0, scan.angle_increment, scan.range_min, scan.range_max)
    hidden = to_laser(pose, 9.5, 6.5)
    assert min(math.dist(hidden, p) for p in points) > 1.0


def test_the_keys_do_not_move_the_robot():
    for config in ("box.json", "box_straight.json"):
        assert load_config(paths.lab_file("config", config))["passthrough"] is False


def test_straight_start_breaks_y_equals_mx_plus_t_and_not_hesse():
    world, cfg = hall("box_straight.json")
    pose = world.spawn_pose(0)
    assert pose.theta == 0.0
    scan = sensors.Lidar(world, sensors.Noise(1), cfg["lidar"]).scan(pose)
    points = G.scan_to_points(scan.ranges, 0.0, scan.angle_increment, scan.range_min, scan.range_max)
    left_wall = [p for p in points if abs(p[0] + 2.0) < 0.05]            # x = 0.5 in the hall
    m, _ = F.fit_line(left_wall)
    alpha, r = F.fit_line_hesse(left_wall)
    assert abs(math.degrees(math.atan(abs(m)))) < 89.0                   # OLS does not find vertical
    assert abs(abs(alpha) - math.pi) < math.radians(0.5) and r == pytest.approx(2.0, abs=0.01)


@pytest.mark.parametrize("name, blocks, call, raised", [
    ("line_fitting", ["0.1", "1.1", "1.2", "1.3", "2.1", "3.1", "4.1", "Z.1"],
     lambda m: m.line_through((0, 0), (1, 1)), "1.1"),
    ("circle_fitting", ["5.1", "5.2", "5.3", "5.4"], lambda m: m.fit_circle([(0, 0), (2, 0), (0, 2)]), "5.1"),
])
def test_the_student_markers_are_balanced_and_the_template_raises(tmp_path, name, blocks, call, raised):
    if not os.path.exists(os.path.join(ROOT, "tools", "make_template.py")):
        pytest.skip("this is the template: tools/ stays with the solution")
    tool = load("tools/make_template.py")
    with open(os.path.join(ROOT, "student", f"{name}.py"), encoding="utf-8") as handle:
        text, ids = tool.strip(handle.read())
    assert ids == blocks
    assert len(text.split("\n")) < 160                                    # short enough to read at once
    (tmp_path / f"{name}.py").write_text(text)
    spec = importlib.util.spec_from_file_location(f"template_{name}", tmp_path / f"{name}.py")
    template = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(template)
    with pytest.raises(NotImplementedError, match=f"STUDENT {raised}"):
        call(template)


def test_the_template_keeps_the_checks_around_the_blocks(tmp_path):
    with open(os.path.join(ROOT, "student", "line_fitting.py"), encoding="utf-8") as handle:
        text = handle.read()
    if os.path.exists(os.path.join(ROOT, "tools", "make_template.py")):      # the solution: strip it first
        text, _ = load("tools/make_template.py").strip(text)
    (tmp_path / "line_fitting.py").write_text(text)
    spec = importlib.util.spec_from_file_location("template_checks", tmp_path / "line_fitting.py")
    template = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(template)
    assert template.line_through((1, 0), (1, 3)) is None                 # given: vertical
    assert template.intersect((0.5, 0), (0.5, 3)) is None                # given: parallel
    assert template.fit_line([(1, 2)]) is None                           # given: too few points
    assert template.num_iterations(1.0, 2) == 1                          # given: w = 1


def test_the_node_is_executable_and_compiles():
    node = os.path.join(ROOT, "student", "polygon_map_node.py")
    assert os.access(node, os.X_OK)
    subprocess.run([sys.executable, "-m", "py_compile", node], check=True)


def test_launch_file_starts_the_simulator_only():
    pytest.importorskip("launch")
    description = load("launch/box.launch.py").generate_launch_description()
    args = {a.name for a in description.entities if hasattr(a, "default_value")}
    assert args == {"robot", "start", "rviz", "headless", "seconds"}


def test_the_rviz_template_names_the_robot_in_every_topic():
    with open(paths.lab_file("rviz", "polygon_map.rviz"), encoding="utf-8") as handle:
        text = handle.read()
    for topic in ("/{robot}/scan", "/{robot}/polygon_map/markers", "/{robot}/polygon_map/room"):
        assert topic in text


def test_straight_start_maps_the_room_with_the_hesse_pipeline():
    world, cfg = hall("box_straight.json")
    pose = world.spawn_pose(0)
    scan = sensors.Lidar(world, sensors.Noise(1), cfg["lidar"]).scan(pose)
    points = G.scan_to_points(scan.ranges, 0.0, scan.angle_increment, scan.range_min, scan.range_max)
    lines, _ = G.find_lines_hesse(points, 4, 0.045, F.num_iterations(0.3, 2), 20, random.Random(1),
                                   F.ransac_line, F.fit_line_hesse)
    corners = G.room_corners([line for line, _ in lines], G.intersect_hesse)
    assert sorted(G.polygon_sides(corners)) == pytest.approx([6.0, 6.0, 9.0, 9.0], abs=0.02)


def test_markers_use_the_newest_transform_not_the_stamp_of_the_scan():
    pytest.importorskip("visualization_msgs")
    from types import SimpleNamespace

    from builtin_interfaces.msg import Time
    from std_msgs.msg import Header

    from ohm_polygon_map.rviz_draw import MapDrawer
    sent = []
    node = SimpleNamespace(create_publisher=lambda *_: SimpleNamespace(publish=sent.append))
    drawer = MapDrawer(node, "/muster/polygon_map")
    drawer.begin(Header(frame_id="muster/laser", stamp=Time(sec=316, nanosec=300000000)))
    drawer.points("scan points", [(1.0, 2.0)])
    drawer.polygon([(0, 0), (1, 0), (1, 1)])
    drawer.end()
    headers = [m.header for msg in sent for m in getattr(msg, "markers", [msg])]
    assert headers and all(h.frame_id == "muster/laser" and h.stamp == Time() for h in headers)


def test_the_rviz_pictures_of_the_handout_are_what_the_node_draws(tmp_path, monkeypatch):
    if not os.path.exists(os.path.join(ROOT, "tools", "make_rviz_figures.py")):
        pytest.skip("this is the template: tools/ stays with the solution")
    pytest.importorskip("visualization_msgs")
    tool = load("tools/make_rviz_figures.py")
    monkeypatch.setattr(tool, "OUT", str(tmp_path))
    with contextlib.redirect_stdout(io.StringIO()):
        tool.main()
    for name in sorted(os.listdir(tmp_path)):
        with open(os.path.join(ROOT, "docs", "figures", name), encoding="utf-8") as handle:
            assert handle.read() == (tmp_path / name).read_text(), f"{name}: python3 tools/make_rviz_figures.py"
