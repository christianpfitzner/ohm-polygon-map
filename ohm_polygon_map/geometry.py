"""Given helpers of the lab 'Polygon map' — nothing to write here, but worth reading.

Same conventions as `student/line_fitting.py` and `student/circle_fitting.py`: points (x, y), lines (m, t) for y = m x + t, circles (x0, y0, r),
all in metres in the frame of the laser. Plain Python, no ROS.
"""
import math


# ----------------------------------------------------------------------------- scan


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """Polar beams of a `sensor_msgs/LaserScan` -> list of (x, y) in the laser frame.

    Beams without an echo arrive as `range_max` (or inf/nan); they are not points and are dropped.
    """
    points = []
    for i, rho in enumerate(ranges):
        if not math.isfinite(rho) or rho < range_min or rho >= range_max:
            continue
        theta = angle_min + i * angle_increment
        points.append((rho * math.cos(theta), rho * math.sin(theta)))
    return points


# ----------------------------------------------------------------------------- lines


def line_distance(point, line):
    """Orthogonal distance of a point to the line y = m x + t: |m x - y + t| / sqrt(1 + m^2)."""
    m, t = line
    return abs(m * point[0] - point[1] + t) / math.sqrt(1.0 + m * m)


def split_line(points, line, tau):
    """(inliers, outliers): the points closer than tau to the line, and the rest."""
    inliers, outliers = [], []
    for p in points:
        (inliers if line_distance(p, line) < tau else outliers).append(p)
    return inliers, outliers


def angle_between(l1, l2):
    """Angle between two lines in degrees, 0 (parallel) ... 90 (perpendicular)."""
    d = abs(math.atan(l1[0]) - math.atan(l2[0]))
    return math.degrees(min(d, math.pi - d))


# ----------------------------------------------------------------------------- polygon


def room_corners(lines, intersect):
    """Corners of the room: `intersect` on every pair of lines, None dropped, ordered around the centroid."""
    corners = []
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            corner = intersect(lines[i], lines[j])
            if corner is not None:
                corners.append(corner)
    if not corners:
        return corners
    cx = sum(c[0] for c in corners) / len(corners)
    cy = sum(c[1] for c in corners) / len(corners)
    return sorted(corners, key=lambda c: math.atan2(c[1] - cy, c[0] - cx))


def polygon_sides(corners):
    """Length of every side of a closed polygon, side i from corner i to corner i + 1."""
    n = len(corners)
    return [math.dist(corners[i], corners[(i + 1) % n]) for i in range(n)]


def polygon_angles(corners):
    """Interior angle at every corner of a closed polygon, in degrees."""
    n, angles = len(corners), []
    for i in range(n):
        (px, py), (cx, cy), (qx, qy) = corners[i - 1], corners[i], corners[(i + 1) % n]
        a = math.atan2(py - cy, px - cx) - math.atan2(qy - cy, qx - cx)
        angles.append(math.degrees(abs(math.atan2(math.sin(a), math.cos(a)))))
    return angles


# ----------------------------------------------------------------------------- circle


def fit_circle(points):
    """Least-squares circle (script eq. 2.37, 2.38), None for < 3 points or points on a straight line.

    Through three points it is the circle through them, so RANSAC uses it for its samples too.
    """
    n = len(points)
    if n < 3:
        return None
    sx = sum(x for x, _ in points)
    sy = sum(y for _, y in points)
    sxx = sum(x * x for x, _ in points)
    syy = sum(y * y for _, y in points)
    sxy = sum(x * y for x, y in points)
    sxxx = sum(x ** 3 for x, _ in points)
    syyy = sum(y ** 3 for _, y in points)
    sxyy = sum(x * y * y for x, y in points)
    sxxy = sum(x * x * y for x, y in points)
    a1 = 2.0 * (sx * sx - n * sxx)
    b1 = a2 = 2.0 * (sx * sy - n * sxy)
    b2 = 2.0 * (sy * sy - n * syy)
    c1 = sx * (sxx + syy) - n * (sxxx + sxyy)
    c2 = sy * (sxx + syy) - n * (sxxy + syyy)
    det = a1 * b2 - a2 * b1
    if abs(det) <= 1e-12 * max(1.0, abs(a1 * b2)):
        return None
    x0 = (c1 * b2 - c2 * b1) / det
    y0 = (a1 * c2 - a2 * c1) / det
    r2 = (sxx - 2.0 * x0 * sx + n * x0 * x0 + syy - 2.0 * y0 * sy + n * y0 * y0) / n
    return x0, y0, math.sqrt(max(r2, 0.0))



# ----------------------------------------------------------------------------- extra task: Hesse normal form


def hesse_distance(point, line):
    """Orthogonal distance of a point to a line (alpha, r): |x cos(alpha) + y sin(alpha) - r|."""
    alpha, r = line
    return abs(point[0] * math.cos(alpha) + point[1] * math.sin(alpha) - r)


def find_lines_hesse(points, n, tau, iterations, min_inliers, rng, ransac_line, fit_line_hesse):
    """find_lines with the least-squares step in Hesse normal form; RANSAC and the fit are yours."""
    rest, lines = list(points), []
    for _ in range(n):
        line, inliers = ransac_line(rest, tau, iterations, rng)
        if line is None or len(inliers) < min_inliers:
            break
        hesse = fit_line_hesse(inliers)
        inliers = [p for p in rest if hesse_distance(p, hesse) < tau]
        hesse = fit_line_hesse(inliers)
        lines.append((hesse, inliers))
        rest = [p for p in rest if hesse_distance(p, hesse) >= tau]
    return lines, rest


def intersect_hesse(l1, l2, min_angle=30.0):
    """Intersection of two lines (alpha, r) by Cramer's rule, None if they meet at less than `min_angle`."""
    (a1, r1), (a2, r2) = l1, l2
    det = math.cos(a1) * math.sin(a2) - math.sin(a1) * math.cos(a2)
    if abs(det) < math.sin(math.radians(min_angle)):
        return None
    return (r1 * math.sin(a2) - r2 * math.sin(a1)) / det, (r2 * math.cos(a1) - r1 * math.cos(a2)) / det
