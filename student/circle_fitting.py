"""Task 5 of the lab 'Polygon map': the least-squares circle and RANSAC for the round object.

    point   (x, y)        metres, in the frame of the laser
    circle  (x0, y0, r)   centre and radius

Replace each `raise NotImplementedError("STUDENT ...")` by your code: the comment above it says in words
what goes there, the task in the handout gives the equation, and the `return` below shows the names to
use. The checks around it are already there.
Plain Python and math, no numpy.
"""
import math


def fit_circle(points):
    """Least-squares circle (script eq. 2.36 to 2.38): (x0, y0, r), None for < 3 points or points on a line.

    Through three points it is the circle through them, so RANSAC uses it for its samples too.
    """
    n = len(points)
    if n < 3:
        return None
    sx = sum(x for x, _ in points)              # the sums of the script: S_x, S_y, S_xx, ...
    sy = sum(y for _, y in points)
    sxx = sum(x * x for x, _ in points)
    syy = sum(y * y for _, y in points)
    sxy = sum(x * y for x, y in points)
    sxxx = sum(x ** 3 for x, _ in points)
    syyy = sum(y ** 3 for _, y in points)
    sxyy = sum(x * y * y for x, y in points)
    sxxy = sum(x * x * y for x, y in points)
    # the coefficients of the linear system for the centre (eq. 2.36), from the sums above
    # TODO STUDENT 5.1
    raise NotImplementedError("STUDENT 5.1")
    det = a1 * b2 - a2 * b1
    if abs(det) <= 1e-12 * max(1.0, abs(a1 * b2)):     # points on a line: there is no circle
        return None
    # the centre from the inverse of the 2x2 matrix (eq. 2.37), then the radius (eq. 2.38)
    # TODO STUDENT 5.2
    raise NotImplementedError("STUDENT 5.2")
    return x0, y0, r


def circle_distance(point, circle):
    """Distance of a point to the circle line."""
    x, y = point
    x0, y0, r = circle
    # how far the point is off the circle line: its distance to the centre against the radius,
    # inside counts the same as outside
    # TODO STUDENT 5.3
    raise NotImplementedError("STUDENT 5.3")
    return d


def ransac_circle(points, tau, iterations, rng, r_max):
    """The circle with the most inliers among `iterations` random three-point samples: (circle, inliers)."""
    best_circle, best_inliers = None, []
    if len(points) < 3:
        return best_circle, best_inliers
    for _ in range(iterations):
        circle = fit_circle(rng.sample(points, 3))      # the circle through three random points
        if circle is None or circle[2] > r_max:         # points on a line, or too big to be the object
            continue
        # score: the points close enough to this circle (no helper here, but task 5.3 gives the distance)
        # keep: as in ransac_line, the circle most points agree with is the new best circle
        # TODO STUDENT 5.4
        raise NotImplementedError("STUDENT 5.4")
    # least squares on all inliers of the best circle, as fit_line did for the walls
    if best_circle is not None:
        refined = fit_circle(best_inliers)
        if refined is not None and refined[2] <= r_max:
            best_circle = refined
            best_inliers = [p for p in points if circle_distance(p, best_circle) < tau]
    return best_circle, best_inliers
