"""Self check of `student/line_fitting.py` and `student/circle_fitting.py`, one group of tests per stage, on made-up points — no ROS needed.

    python3 -m pytest test/test_fitting.py -q              all stages
    python3 -m pytest test/test_fitting.py -q -k stage1    one stage
    python3 -m pytest test/test_fitting.py -q -k extra     the extra task Z.1
"""
import math
import os
import random
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [ROOT, os.path.join(ROOT, "student")]
import circle_fitting as C                                                 # noqa: E402
import line_fitting as F                                                   # noqa: E402
from ohm_polygon_map import geometry as G                                  # noqa: E402


def on_line(m, t, xs, noise=0.0, seed=0):
    """Points of y = m x + t at the given x, moved by Gaussian noise perpendicular to the line."""
    rng = random.Random(seed)
    nx, ny = -m / math.sqrt(1 + m * m), 1 / math.sqrt(1 + m * m)
    out = []
    for x in xs:
        d = rng.gauss(0, noise)
        out.append((x + d * nx, m * x + t + d * ny))
    return out


def on_segment(a, b, step=0.05, noise=0.0, seed=0):
    """Points from a to b, every `step` metres, with noise perpendicular to the segment."""
    rng = random.Random(seed)
    length = math.dist(a, b)
    ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
    n = int(length / step)
    return [(a[0] + ux * s - uy * d, a[1] + uy * s + ux * d)
            for s, d in ((length * (k + 0.5) / n, rng.gauss(0, noise)) for k in range(n))]


def on_circle(x0, y0, r, angles, noise=0.0, seed=0):
    rng = random.Random(seed)
    return [(x0 + (r + rng.gauss(0, noise)) * math.cos(a), y0 + (r + rng.gauss(0, noise)) * math.sin(a))
            for a in angles]


def flat(points):
    return [v for p in points for v in p]


def rectangle_scan(noise=0.01, seed=0):
    """A 6 x 4 m room turned by 25 degrees around a sensor inside it: (corners, points)."""
    c, s = math.cos(math.radians(25)), math.sin(math.radians(25))
    corners = [(c * x - s * y, s * x + c * y) for x, y in [(-1.5, -1.0), (4.5, -1.0), (4.5, 3.0), (-1.5, 3.0)]]
    points = []
    for i in range(4):
        points += on_segment(corners[i], corners[(i + 1) % 4], noise=noise, seed=seed + i)
    return corners, points


# ----------------------------------------------------------------------------- stage 0


def test_stage0_mean_and_std():
    mean, std = F.mean_and_std([1.0, 2.0, 3.0, 4.0])
    assert mean == pytest.approx(2.5)
    assert std == pytest.approx(math.sqrt(5.0 / 3.0))                 # n - 1 in the denominator


def test_stage0_mean_and_std_of_a_constant():
    assert F.mean_and_std([4.2] * 10) == pytest.approx((4.2, 0.0))


# ----------------------------------------------------------------------------- stage 1


def test_stage1_line_through_two_points():
    assert F.line_through((0.0, 1.0), (2.0, 5.0)) == pytest.approx((2.0, 1.0))
    assert F.line_through((2.0, 5.0), (0.0, 1.0)) == pytest.approx((2.0, 1.0))      # order does not matter
    assert F.line_through((1.0, -2.0), (4.0, -2.0)) == pytest.approx((0.0, -2.0))


def test_stage1_no_line_through_two_points_above_each_other():
    assert F.line_through((1.0, 0.0), (1.0, 3.0)) is None


def test_stage1_num_iterations_as_in_the_lecture_exercise():
    assert F.num_iterations(0.8, 2) == 5
    assert F.num_iterations(0.5, 2) == 17
    assert F.num_iterations(0.5, 3) == 35
    assert F.num_iterations(1.0, 2) == 1


def test_stage1_ransac_finds_the_line_among_outliers():
    rng = random.Random(1)
    inliers = on_line(0.6, 1.5, [i * 0.05 - 2.0 for i in range(80)], noise=0.01, seed=1)
    outliers = [(rng.uniform(-4, 4), rng.uniform(-4, 4)) for _ in range(60)]
    line, found = F.ransac_line(inliers + outliers, 0.03, F.num_iterations(0.5, 2, 0.999), rng)
    assert line == pytest.approx((0.6, 1.5), abs=0.05)
    assert len(found) >= 75


def test_stage1_ransac_with_too_few_points():
    assert F.ransac_line([(1.0, 1.0)], 0.03, 10, random.Random(1)) == (None, [])


# ----------------------------------------------------------------------------- stage 2


def test_stage2_fit_line_exact_points():
    for m, t in ((0.0, 2.0), (0.5, -1.0), (-2.7, 3.3), (10.0, 0.0)):
        assert F.fit_line(on_line(m, t, [-1.0, -0.3, 0.4, 2.0])) == pytest.approx((m, t)), (m, t)


def test_stage2_fit_line_as_in_the_lecture_exercise():
    points = [(0.0, 1.1), (1.0, 2.9), (2.0, 5.2), (3.0, 6.8)]
    assert F.fit_line(points) == pytest.approx((1.94, 1.09))


def test_stage2_least_squares_beats_the_two_point_line():
    rng = random.Random(3)
    points = on_line(-0.8, 3.0, [i * 0.04 - 3 for i in range(150)], noise=0.015, seed=3)
    sample = F.line_through(*rng.sample(points, 2))
    inliers, _ = G.split_line(points, sample, 0.045)
    refined = F.fit_line(inliers)
    assert len(inliers) >= 140
    err = lambda line: abs(line[0] + 0.8) + abs(line[1] - 3.0)     # noqa: E731
    assert err(refined) < 0.01
    assert err(refined) <= err(sample) + 1e-9


def test_stage2_fit_line_needs_two_different_x():
    assert F.fit_line([(1.0, 2.0)]) is None
    assert F.fit_line([(1.0, 2.0), (1.0, 3.0)]) is None


# ----------------------------------------------------------------------------- stage 3


def test_stage3_four_walls_of_a_room():
    corners, points = rectangle_scan()
    lines, rest = F.find_lines(points, 4, 0.03, F.num_iterations(0.25, 2), 20, random.Random(2))
    assert len(lines) == 4
    for i in range(4):
        wall = F.line_through(corners[i], corners[(i + 1) % 4])
        assert any(G.angle_between(found, wall) < 0.5 and G.line_distance(corners[i], found) < 0.01
                   for found, _ in lines), i
    assert sum(len(inl) for _, inl in lines) + len(rest) == len(points)
    assert len(rest) < 10


def test_stage3_stops_when_no_line_has_enough_points():
    _, points = rectangle_scan()
    lines, _ = F.find_lines(points, 6, 0.03, 60, 20, random.Random(2))
    assert len(lines) == 4


# ----------------------------------------------------------------------------- stage 4


def test_stage4_intersect_two_lines():
    assert F.intersect((1.0, 0.0), (-1.0, 2.0)) == pytest.approx((1.0, 1.0))
    assert F.intersect((0.0, 3.0), (2.0, -1.0)) == pytest.approx((2.0, 3.0))


def test_stage4_parallel_lines_do_not_intersect():
    assert F.intersect((0.5, 0.0), (0.5, 3.0)) is None
    assert F.intersect((0.0, 0.0), (0.1, 3.0)) is None                 # 6 degrees: as good as parallel


def test_stage4_room_corners_are_four_and_in_order():
    corners, points = rectangle_scan(noise=0.0)
    lines = [F.line_through(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    found = G.room_corners(lines, F.intersect)
    assert len(found) == 4
    assert sorted(G.polygon_sides(found)) == pytest.approx([4.0, 4.0, 6.0, 6.0])
    assert G.polygon_angles(found) == pytest.approx([90.0] * 4)


# ----------------------------------------------------------------------------- stage 5


def test_stage5_circle_through_three_points_as_in_the_lecture_exercise():
    assert C.fit_circle([(0.0, 0.0), (2.0, 0.0), (0.0, 2.0)]) == pytest.approx((1.0, 1.0, math.sqrt(2.0)))


def test_stage5_no_circle_through_points_on_a_line():
    assert C.fit_circle([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]) is None
    assert C.fit_circle([(0.0, 0.0), (1.0, 1.0)]) is None


def test_stage5_least_squares_circle_on_a_noisy_arc():
    arc = on_circle(4.0, 1.0, 0.8, [math.radians(a) for a in range(120, 240, 5)], noise=0.005, seed=2)
    assert C.fit_circle(arc) == pytest.approx((4.0, 1.0, 0.8), abs=0.02)


def test_stage5_circle_distance():
    assert C.circle_distance((3.0, 0.0), (0.0, 0.0, 1.0)) == pytest.approx(2.0)
    assert C.circle_distance((0.0, 0.5), (0.0, 0.0, 1.0)) == pytest.approx(0.5)
    assert C.circle_distance((1.0, 2.0), (1.0, 1.0, 1.0)) == pytest.approx(0.0)


def test_stage5_ransac_finds_the_circle_among_outliers():
    rng = random.Random(4)
    arc = on_circle(4.0, 1.0, 0.8, [math.radians(a) for a in range(120, 240, 5)], noise=0.01, seed=4)
    clutter = [(rng.uniform(0, 8), rng.uniform(-3, 3)) for _ in range(20)]
    circle, inliers = C.ransac_circle(arc + clutter, 0.03, F.num_iterations(0.5, 3, 0.999), rng, 2.0)
    assert circle == pytest.approx((4.0, 1.0, 0.8), abs=0.03)
    assert len(inliers) >= len(arc) - 1


def test_stage5_huge_circles_are_not_objects():
    points = on_line(0.0, 3.0, [i * 0.1 for i in range(30)])
    circle, _ = C.ransac_circle(points, 0.03, 50, random.Random(1), 2.0)
    assert circle is None or circle[2] <= 2.0




# ----------------------------------------------------------------------------- extra task Z.1


def test_extra_hesse_fit_exact_points():
    for alpha in (0.0, 0.7, math.pi / 2, 2.5, -2.0, -math.pi / 2):
        r = 1.7
        fx, fy, dx, dy = r * math.cos(alpha), r * math.sin(alpha), -math.sin(alpha), math.cos(alpha)
        points = [(fx + s * dx, fy + s * dy) for s in (-1.0, -0.3, 0.4, 2.0)]
        a, rr = F.fit_line_hesse(points)
        assert abs(math.atan2(math.sin(a - alpha), math.cos(a - alpha))) < 1e-9 and rr == pytest.approx(r)


def test_extra_hesse_handles_the_vertical_wall_that_y_equals_mx_plus_t_cannot():
    points = on_segment((2.0, -2.0), (2.0, 3.0), noise=0.015, seed=5)
    alpha, r = F.fit_line_hesse(points)
    assert abs(alpha) < math.radians(1.0) and r == pytest.approx(2.0, abs=0.01)
    m, _ = F.fit_line(points)
    assert abs(math.degrees(math.atan(abs(m)))) < 85.0                     # OLS tilts the wall
