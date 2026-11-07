#!/usr/bin/env python3
"""Lab "Polygon map": a map of the room from one laser scan — four walls, four corners, one circle.

    ros2 launch ohm_polygon_map box.launch.py                                        # terminal 1
    ros2 run ohm_polygon_map polygon_map_node.py --ros-args -p stage:=1             # terminal 2

The node is given; the algorithms it calls are yours, in `line_fitting.py` and `circle_fitting.py` next
to this file. `stage` says how far the pipeline runs:

    0  noise: one beam over `noise_scans` scans -> mean, sigma, tau = 3 sigma     (always first)
    1  RANSAC: one line, any line
    2  least squares: the RANSAC line refined on its inliers, y = m x + t
    3  four lines: sequential RANSAC, every wall an outlier for the other three
    4  corners: the intersections of the four lines -> the room polygon
    5  circle: RANSAC with three-point samples on what the walls left over

`-p hesse:=true` (extra task Z.1) runs stages 3 and 4 with lines in Hesse normal form instead of y = m x + t.

Results go to the terminal and to RViz (/<robot>/polygon_map/markers, /<robot>/polygon_map/room).
"""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.realpath(__file__))              # student/, also via the installed symlink
for path in (HERE, os.path.dirname(HERE)):                     # your files, then the package next door
    if path not in sys.path:
        sys.path.insert(0, path)

import rclpy                                                  # noqa: E402
from rclpy.executors import ExternalShutdownException         # noqa: E402
from rclpy.node import Node                                   # noqa: E402
from sensor_msgs.msg import LaserScan                         # noqa: E402

import circle_fitting                                         # noqa: E402
import line_fitting                                           # noqa: E402
from ohm_polygon_map import geometry                          # noqa: E402
from ohm_polygon_map.rviz_draw import GREY, RED, WALL_COLORS, WHITE, MapDrawer   # noqa: E402

PARAMETERS = {
    "robot": "muster",          # topics /<robot>/scan and /<robot>/polygon_map/...
    "stage": 5,                 # how far the pipeline runs, see above
    "noise_beam": 250,          # the beam measured in stage 0: from the start pose straight at the lower wall
    "noise_scans": 100,         # how many scans stage 0 collects (20 per second)
    "tau": 0.0,                 # inlier threshold in m; 0 = 3 sigma from stage 0
    "p": 0.99,                  # RANSAC: probability of at least one all-inlier sample
    "w_line": 0.3,              # RANSAC: inlier ratio assumed for a line (worst case of the four)
    "w_circle": 0.5,            # RANSAC: inlier ratio assumed for the circle, after the walls are gone
    "min_inliers": 20,          # a line or circle with fewer points is not accepted
    "r_max": 2.0,               # largest circle radius that counts as an object, m
    "seed": 1,                  # random seed of RANSAC: the same seed, the same samples
    "period": 2.0,              # s between two evaluations
    "hesse": False,             # extra task Z.1: stages 3 and 4 in Hesse normal form
}


class PolygonMapNode(Node):

    def __init__(self):
        super().__init__("polygon_map")
        self.p = {name: self.declare_parameter(name, value).value for name, value in PARAMETERS.items()}
        robot = self.p["robot"]
        self.create_subscription(LaserScan, f"/{robot}/scan", self.on_scan, 10)
        self.drawer = MapDrawer(self, f"/{robot}/polygon_map")
        self.create_timer(float(self.p["period"]), self.update)
        self.scan = None
        self.samples = []
        self.tau = float(self.p["tau"]) or None
        self.get_logger().info(f"stage {self.p['stage']}, listening on /{robot}/scan")

    # ------------------------------------------------------------------ stage 0

    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg
        if self.tau is not None:
            return
        self.samples.append(msg.ranges[int(self.p["noise_beam"])])
        if len(self.samples) < int(self.p["noise_scans"]):
            return
        mean, std = line_fitting.mean_and_std(self.samples)
        self.tau = 3.0 * std
        self.get_logger().info(f"[0] beam {self.p['noise_beam']} over {len(self.samples)} scans: "
                               f"mean {mean:.4f} m, sigma {std * 1000:.1f} mm -> tau = 3 sigma = "
                               f"{self.tau * 1000:.1f} mm")

    # ------------------------------------------------------------------ stages 1-5

    def update(self) -> None:
        stage = int(self.p["stage"])
        if self.scan is None or self.tau is None or stage < 1:
            return
        s, tau, log = self.scan, self.tau, self.get_logger().info
        rng = random.Random(int(self.p["seed"]))
        points = geometry.scan_to_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)
        k_line = line_fitting.num_iterations(float(self.p["w_line"]), 2, float(self.p["p"]))
        self.drawer.begin(s.header)
        self.drawer.points("scan points", points, GREY)

        if stage <= 2:
            line, inliers = line_fitting.ransac_line(points, tau, k_line, rng)
            log(f"[1] {len(points)} points, k = {k_line}: RANSAC line {fmt_line(line)}, "
                f"{len(inliers)} inliers")
            self.drawer.points("inliers", inliers, RED, size=0.07)
            if stage == 1:
                self.drawer.line("RANSAC line", line, RED)
            else:
                refined = line_fitting.fit_line(inliers)
                inliers, _ = geometry.split_line(points, refined, tau)
                log(f"[2] least squares {fmt_line(refined)}, {len(inliers)} inliers")
                self.drawer.line("RANSAC line", line, GREY, width=0.02)
                self.drawer.line("LS line", refined, RED)
            self.drawer.end()
            return

        hesse = bool(self.p["hesse"])
        if hesse:
            lines, rest = geometry.find_lines_hesse(points, 4, tau, k_line, int(self.p["min_inliers"]), rng,
                                                    line_fitting.ransac_line, line_fitting.fit_line_hesse)
        else:
            lines, rest = line_fitting.find_lines(points, 4, tau, k_line, int(self.p["min_inliers"]), rng)
        log(f"[3] {len(points)} points: {len(lines)} lines{' (Hesse)' if hesse else ''} "
            f"(k = {k_line} each), {len(rest)} points left over")
        for i, (line, inliers) in enumerate(lines):
            log(f"    wall {i + 1}: {fmt_hesse(line) if hesse else fmt_line(line)}, {len(inliers)} inliers")
            color = WALL_COLORS[i % len(WALL_COLORS)]
            self.drawer.points(f"wall {i + 1}", inliers, color, size=0.07)
            (self.drawer.line_hesse if hesse else self.drawer.line)(f"wall {i + 1}", line, color,
                                                                     along=inliers, width=0.03)

        if stage >= 4:
            corners = geometry.room_corners([line for line, _ in lines],
                                            geometry.intersect_hesse if hesse else line_fitting.intersect)
            sides, angles = geometry.polygon_sides(corners), geometry.polygon_angles(corners)
            log(f"[4] {len(corners)} corners: " + ", ".join(f"({x:.3f}, {y:.3f})" for x, y in corners))
            log("    sides " + ", ".join(f"{d:.3f}" for d in sides) + " m, angles "
                + ", ".join(f"{a:.1f}" for a in angles) + " deg")
            self.drawer.polygon(corners, RED)
            self.drawer.corners("corners", corners, RED)
            for i, d in enumerate(sides):
                (x0, y0), (x1, y1) = corners[i], corners[(i + 1) % len(corners)]
                self.drawer.text("sides", ((x0 + x1) / 2, (y0 + y1) / 2), f"{d:.2f} m", WHITE)

        if stage >= 5:
            k_circle = line_fitting.num_iterations(float(self.p["w_circle"]), 3, float(self.p["p"]))
            circle, inliers = circle_fitting.ransac_circle(rest, tau, k_circle, rng, float(self.p["r_max"]))
            if circle is None or len(inliers) < int(self.p["min_inliers"]):
                log(f"[5] no circle in {len(rest)} points (k = {k_circle})")
            else:
                log(f"[5] circle centre ({circle[0]:.3f}, {circle[1]:.3f}), r = {circle[2]:.3f} m, "
                    f"{len(inliers)} of {len(rest)} points (k = {k_circle})")
                self.drawer.points("circle", inliers, RED, size=0.07)
                self.drawer.circle("circle", circle, RED)
                self.drawer.text("circle", circle[:2], f"r {circle[2]:.2f} m", WHITE)
        self.drawer.end()


def fmt_line(line) -> str:
    if line is None:
        return "none"
    m, t = line
    return f"y = {m:7.3f} x + {t:6.3f} ({math.degrees(math.atan(m)):6.1f} deg)"


def fmt_hesse(line) -> str:
    return "none" if line is None else f"alpha {math.degrees(line[0]):7.2f} deg, r {line[1]:.3f} m"


def main():
    rclpy.init()
    node = PolygonMapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
