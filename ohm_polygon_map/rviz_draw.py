"""Drawing the results of `student/line_fitting.py` and `circle_fitting.py` in RViz — given, nothing to write here.

One `MapDrawer` per node. Every update is one frame:

    drawer.begin(scan.header)            # the frame of the scan the points came from
    drawer.points("scan", points, GREY)
    drawer.line("wall 1", line, RED, along=inliers)
    drawer.polygon(corners)
    drawer.circle("circle", circle, RED)
    drawer.end()                         # publishes; the previous frame disappears

Everything is drawn in the frame of the scan, with the conventions of the student files: points (x, y),
lines (m, t) for y = m x + t, circles (x0, y0, r). `line_hesse` draws a line (alpha, r) of the extra task.

Topics (prefix = the `prefix` argument, e.g. /muster/polygon_map):
    <prefix>/markers   visualization_msgs/MarkerArray   points, lines, corners, circle, labels
    <prefix>/room      geometry_msgs/PolygonStamped     the room polygon
"""
import math

from geometry_msgs.msg import Point, Point32, PolygonStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

GREY = (0.6, 0.6, 0.6, 1.0)
RED = (0.78, 0.06, 0.18, 1.0)              # OHM red
BLUE = (0.12, 0.37, 0.66, 1.0)
GREEN = (0.20, 0.63, 0.29, 1.0)
ORANGE = (0.95, 0.55, 0.10, 1.0)
PURPLE = (0.55, 0.30, 0.70, 1.0)
WHITE = (1.0, 1.0, 1.0, 1.0)
WALL_COLORS = (BLUE, GREEN, ORANGE, PURPLE)


class MapDrawer:
    """Collects markers for one frame and publishes them as one MarkerArray."""

    def __init__(self, node, prefix: str):
        self.pub_markers = node.create_publisher(MarkerArray, f"{prefix}/markers", 10)
        self.pub_room = node.create_publisher(PolygonStamped, f"{prefix}/room", 10)
        self.header = None
        self.markers = []

    # ------------------------------------------------------------------ frame

    def begin(self, header) -> None:
        """Start a new frame in the frame of `header` (the scan's).

        The stamp is left at 0, "the newest transform": the robot stands, and the simulator sends a scan
        just before the /tf of the same instant. With the scan's own stamp RViz would often find no
        transform yet, and the MarkerArray display does not wait for one.
        """
        self.header = Header(frame_id=header.frame_id)
        clear = Marker()
        clear.header = self.header
        clear.action = Marker.DELETEALL
        self.markers = [clear]

    def end(self) -> None:
        """Publish the frame."""
        self.pub_markers.publish(MarkerArray(markers=self.markers))
        self.markers = []

    # ------------------------------------------------------------------ shapes

    def points(self, ns: str, points, color=GREY, size: float = 0.05) -> None:
        """Scan points (or any points) as small squares."""
        m = self._marker(ns, Marker.POINTS, color)
        m.scale.x = m.scale.y = size
        m.points = [Point(x=float(x), y=float(y)) for x, y in points]
        self.markers.append(m)

    def line(self, ns: str, line, color=RED, along=None, length: float = 12.0,
             width: float = 0.03) -> None:
        """A line (m, t), y = m x + t, as a segment.

        `along`: points on the line (its inliers); the segment then runs from the first to the last of
        them. Without it the segment is `length` long, centred on the foot point closest to the laser.
        """
        if line is None:
            return
        m, t = line
        norm = math.sqrt(1.0 + m * m)
        self._segment(ns, (-m * t / norm ** 2, t / norm ** 2), (1.0 / norm, m / norm), color, along,
                      length, width)

    def line_hesse(self, ns: str, line, color=WHITE, along=None, length: float = 12.0,
                   width: float = 0.02) -> None:
        """A line (alpha, r) in Hesse normal form, x cos(alpha) + y sin(alpha) = r, as a segment."""
        if line is None:
            return
        alpha, r = line
        self._segment(ns, (r * math.cos(alpha), r * math.sin(alpha)), (-math.sin(alpha), math.cos(alpha)),
                      color, along, length, width)

    def _segment(self, ns, foot, direction, color, along, length, width) -> None:
        (fx, fy), (dx, dy) = foot, direction
        if along:
            ts = [(x - fx) * dx + (y - fy) * dy for x, y in along]
            t0, t1 = min(ts), max(ts)
        else:
            t0, t1 = -length / 2, length / 2
        self.segments(ns, [((fx + t0 * dx, fy + t0 * dy), (fx + t1 * dx, fy + t1 * dy))], color, width)

    def segments(self, ns: str, pairs, color=RED, width: float = 0.03) -> None:
        """Any number of segments ((x0, y0), (x1, y1))."""
        m = self._marker(ns, Marker.LINE_LIST, color)
        m.scale.x = width
        for a, b in pairs:
            m.points += [Point(x=float(a[0]), y=float(a[1])), Point(x=float(b[0]), y=float(b[1]))]
        self.markers.append(m)

    def corners(self, ns: str, corners, color=RED, size: float = 0.15) -> None:
        """Corners as spheres."""
        m = self._marker(ns, Marker.SPHERE_LIST, color)
        m.scale.x = m.scale.y = m.scale.z = size
        m.points = [Point(x=float(x), y=float(y)) for x, y in corners]
        self.markers.append(m)

    def polygon(self, corners, color=RED, width: float = 0.05) -> None:
        """The room: a closed outline in the MarkerArray and a PolygonStamped on <prefix>/room."""
        if len(corners) < 2:
            return
        m = self._marker("room", Marker.LINE_STRIP, color)
        m.scale.x = width
        m.points = [Point(x=float(x), y=float(y)) for x, y in list(corners) + [corners[0]]]
        self.markers.append(m)
        msg = PolygonStamped()
        msg.header = self.header
        msg.polygon.points = [Point32(x=float(x), y=float(y)) for x, y in corners]
        self.pub_room.publish(msg)

    def circle(self, ns: str, circle, color=RED, width: float = 0.04) -> None:
        """A circle (x0, y0, r) as an outline and a transparent disc."""
        if circle is None:
            return
        x0, y0, r = circle
        ring = self._marker(ns, Marker.LINE_STRIP, color)
        ring.scale.x = width
        ring.points = [Point(x=x0 + r * math.cos(2 * math.pi * i / 64),
                             y=y0 + r * math.sin(2 * math.pi * i / 64)) for i in range(65)]
        disc = self._marker(ns, Marker.CYLINDER, color[:3] + (0.3,))
        disc.pose.position.x, disc.pose.position.y = float(x0), float(y0)
        disc.scale.x = disc.scale.y = 2.0 * r
        disc.scale.z = 0.02
        self.markers += [ring, disc]

    def text(self, ns: str, xy, text: str, color=WHITE, height: float = 0.25) -> None:
        """A label at (x, y), always facing the camera."""
        m = self._marker(ns, Marker.TEXT_VIEW_FACING, color)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(xy[0]), float(xy[1]), 0.3
        m.scale.z = height
        m.text = text
        self.markers.append(m)

    # ------------------------------------------------------------------ inside

    def _marker(self, ns: str, kind: int, color) -> Marker:
        m = Marker()
        m.header = self.header
        m.ns = ns
        m.id = len(self.markers)
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = (float(c) for c in color)
        return m
