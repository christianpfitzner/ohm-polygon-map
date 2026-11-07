"""Tasks 0 to 4 of the lab 'Polygon map': noise, RANSAC and least squares for the walls.

    point   (x, y)    metres, in the frame of the laser
    line    (m, t)    y = m x + t

Write the lines between `# >>> STUDENT` and `# <<< STUDENT`. The checks around them are already there.
Helpers (split_line, angle_between) are in ohm_polygon_map/geometry.py. Plain Python and math, no numpy.
"""
import math

from ohm_polygon_map.geometry import angle_between, split_line


# task 0: noise --------------------------------------------------------------------------------------


def mean_and_std(values):
    """Mean (script eq. 2.18) and sample standard deviation of a list of numbers."""
    n = len(values)
    # mean:  sum of all values divided by n
    # std:   square root of  sum of (value - mean)^2  divided by (n - 1)
    # TODO STUDENT 0.1
    raise NotImplementedError("STUDENT 0.1")
    return mean, std


# task 1: RANSAC -------------------------------------------------------------------------------------


def line_through(p, q):
    """The line y = m x + t through the points p = (x1, y1) and q = (x2, y2)."""
    dx = q[0] - p[0]
    if abs(dx) < 1e-9:              # p and q above each other: a vertical line has no m
        return None
    # m = (y2 - y1) / (x2 - x1),   then t from  y1 = m x1 + t
    # TODO STUDENT 1.1
    raise NotImplementedError("STUDENT 1.1")
    return m, t


def num_iterations(w, s, p=0.99):
    """RANSAC iterations k for inlier ratio w, sample size s and success probability p (script eq. 2.30)."""
    if w >= 1.0:                    # every point is an inlier: one sample is enough
        return 1
    # k = log(1 - p) / log(1 - w^s), rounded up with math.ceil
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
        # inliers, outliers = split_line(points, line, tau)
        # more inliers than the best line so far?  then this line is the new best line
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
    # m = sum of (x - mx)(y - my)  divided by  sum of (x - mx)^2       (eq. 2.23)
    # t = my - m mx                                                     (eq. 2.24)
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
        # 3. line = fit_line(inliers)                    least squares on the inliers of RANSAC
        # 4. inliers, rest = split_line(rest, line, tau)  the wall's points, and the points left over
        # 5. lines.append((line, inliers))
        # TODO STUDENT 3.1
        raise NotImplementedError("STUDENT 3.1")
    return lines, rest


# task 4: corners ------------------------------------------------------------------------------------


def intersect(l1, l2, min_angle=30.0):
    """The corner (x, y) where two lines meet."""
    if angle_between(l1, l2) < min_angle:      # (nearly) parallel: opposite walls have no corner
        return None
    (m1, t1), (m2, t2) = l1, l2
    # x from  m1 x + t1 = m2 x + t2,   then y on one of the two lines
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
    # q_x, q_y, q_xy with eq. 2.25 (sums over all points, like in fit_line)
    # alpha = atan2(-2 q_xy, q_y - q_x) / 2,   r = mx cos(alpha) + my sin(alpha)     (eq. 2.29)
    # TODO STUDENT Z.1
    raise NotImplementedError("STUDENT Z.1")
    if r < 0.0:                         # normal points away from the laser: turn it round
        alpha, r = alpha + math.pi, -r
    return math.atan2(math.sin(alpha), math.cos(alpha)), r
