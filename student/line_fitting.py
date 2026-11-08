"""Tasks 0 to 4 of the lab 'Polygon map': noise, RANSAC and least squares for the walls.

    point   (x, y)    metres, in the frame of the laser
    line    (m, t)    y = m x + t

Replace each `raise NotImplementedError("STUDENT ...")` by your code: the comment above it says in words
what goes there, the task in the handout gives the equation, and the `return` below shows the names to
use. The checks around it are already there.
Helpers (split_line, angle_between) are in ohm_polygon_map/geometry.py. Plain Python and math, no numpy.
"""
import math

from ohm_polygon_map.geometry import angle_between, split_line


# task 0: noise --------------------------------------------------------------------------------------


def mean_and_std(values):
    """Mean (script eq. 2.18) and sample standard deviation of a list of numbers."""
    n = len(values)
    # the mean of the values (eq. 2.18), and the sample standard deviation: how far they scatter around it
    # TODO STUDENT 0.1
    raise NotImplementedError("STUDENT 0.1")
    return mean, std


# task 1: RANSAC -------------------------------------------------------------------------------------


def line_through(p, q):
    """The line y = m x + t through the points p = (x1, y1) and q = (x2, y2)."""
    dx = q[0] - p[0]
    if abs(dx) < 1e-9:              # p and q above each other: a vertical line has no m
        return None
    # the slope from the two points, then the offset from one point that lies on the line
    # TODO STUDENT 1.1
    raise NotImplementedError("STUDENT 1.1")
    return m, t


def num_iterations(w, s, p=0.99):
    """RANSAC iterations k for inlier ratio w, sample size s and success probability p (script eq. 2.30)."""
    if w >= 1.0:                    # every point is an inlier: one sample is enough
        return 1
    # the number of samples from eq. 2.30, rounded up to a whole number
    # TODO STUDENT 1.2
    raise NotImplementedError("STUDENT 1.2")
    return k


def ransac_line(points, tau, iterations, rng):
    """The line with the most inliers among `iterations` random two-point samples: (line, inliers)."""
    best_line, best_inliers = None, []
    if len(points) < 2:
        return best_line, best_inliers
    for _ in range(iterations):
        p, q = rng.sample(points, 2)            # two different random points
        line = line_through(p, q)
        if line is None:
            continue
        # score: which points lie close to this line; split_line in geometry.py sorts them and returns
        #        a pair (inliers, outliers), so unpack it
        # keep: if more points agree with this line than with the best one so far, it is the new best
        # TODO STUDENT 1.3
        raise NotImplementedError("STUDENT 1.3")
    return best_line, best_inliers


# task 2: least squares ------------------------------------------------------------------------------


def fit_line(points):
    """Ordinary least squares y = m x + t through all points (script eq. 2.23, 2.24)."""
    n = len(points)
    xs = [x for x, _ in points]
    if n < 2 or max(xs) - min(xs) < 1e-9:      # too few points, or all on one vertical line
        return None
    mx = sum(x for x, _ in points) / n          # mean of x
    my = sum(y for _, y in points) / n          # mean of y
    # slope (eq. 2.23) and offset (eq. 2.24) of the line with the smallest squared errors;
    # the means of both coordinates are already above
    # TODO STUDENT 2.1
    raise NotImplementedError("STUDENT 2.1")
    return m, t


# task 3: four lines ---------------------------------------------------------------------------------


def find_lines(points, n, tau, iterations, min_inliers, rng):
    """Sequential RANSAC: up to n walls. Returns (lines, rest).

    lines: a list of (line, inliers), one per wall;  rest: the points no wall explains.
    """
    rest = list(points)                 # the points no wall has taken yet
    lines = []
    for _ in range(n):
        # 1. RANSAC on the points that are left
        line, inliers = ransac_line(rest, tau, iterations, rng)
        # 2. too few inliers: there is no further wall
        if line is None or len(inliers) < min_inliers:
            break
        # 3. make the rough line of RANSAC exact: least squares on its inliers (task 2)
        # 4. with the exact line, part the points that are left into this wall's points and the rest
        # 5. keep the wall together with its points
        # TODO STUDENT 3.1
        raise NotImplementedError("STUDENT 3.1")
    return lines, rest


# task 4: corners ------------------------------------------------------------------------------------


def intersect(l1, l2, min_angle=30.0):
    """The corner (x, y) where two lines meet."""
    if angle_between(l1, l2) < min_angle:      # (nearly) parallel: opposite walls have no corner
        return None
    (m1, t1), (m2, t2) = l1, l2
    # the corner lies on both lines: first where along the axis they are equally high, then that height
    # TODO STUDENT 4.1
    raise NotImplementedError("STUDENT 4.1")
    return x, y


# extra task Z.1: Hesse normal form ------------------------------------------------------------------


def fit_line_hesse(points):
    """Orthogonal least squares, x cos(alpha) + y sin(alpha) = r (script eq. 2.25, 2.29): (alpha, r)."""
    n = len(points)
    if n < 2:
        return None
    mx = sum(x for x, _ in points) / n
    my = sum(y for _, y in points) / n
    # the three sums of deviations from the means (eq. 2.25), then the angle of the normal and the
    # distance of the line from the laser (eq. 2.29)
    # TODO STUDENT Z.1
    raise NotImplementedError("STUDENT Z.1")
    if r < 0.0:                         # normal points away from the laser: turn it round
        alpha, r = alpha + math.pi, -r
    return math.atan2(math.sin(alpha), math.cos(alpha)), r
