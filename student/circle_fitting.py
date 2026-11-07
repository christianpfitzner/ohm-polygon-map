"""Task 5 of the lab 'Polygon map': RANSAC for the round object.

    point   (x, y)        metres, in the frame of the laser
    circle  (x0, y0, r)   centre and radius

Write the lines between `# >>> STUDENT` and `# <<< STUDENT`. The checks around them are already there.
fit_circle (script eq. 2.37, 2.38) is in ohm_polygon_map/geometry.py. Plain Python and math, no numpy.
"""
import math

from ohm_polygon_map.geometry import fit_circle


def circle_distance(point, circle):
    """Distance of a point to the circle line."""
    x, y = point
    x0, y0, r = circle
    # | sqrt((x - x0)^2 + (y - y0)^2) - r |      (script, RANSAC for circles: score)
    # TODO STUDENT 5.1
    raise NotImplementedError("STUDENT 5.1")
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
        # inliers: the points with circle_distance(point, circle) < tau
        # more inliers than the best circle so far?  then this circle is the new best circle
        # (the same as in ransac_line)
        # TODO STUDENT 5.2
        raise NotImplementedError("STUDENT 5.2")
    # least squares on all inliers of the best circle, as fit_line did for the walls
    if best_circle is not None:
        refined = fit_circle(best_inliers)
        if refined is not None and refined[2] <= r_max:
            best_circle = refined
            best_inliers = [p for p in points if circle_distance(p, best_circle) < tau]
    return best_circle, best_inliers
