import numpy as np


class CapsuleCollider:
    """
    Capsule collision shape defined by a line segment AB and a radius.

    @ivar pointA: Start point of the capsule axis.
    @type pointA: numpy.ndarray
    @ivar pointB: End point of the capsule axis.
    @type pointB: numpy.ndarray
    @ivar radius: Capsule radius in meters.
    @type radius: float
    """

    def __init__(self, pointA, pointB, radius: float):
        """
        @param pointA: Start point of the capsule axis.
        @param pointB: End point of the capsule axis.
        @param radius: Capsule radius in meters.
        @type radius: float
        """
        self.pointA = np.array(pointA, dtype=float)
        self.pointB = np.array(pointB, dtype=float)
        self.radius = radius

    def closestPoint(self, point) -> np.ndarray:
        """
        Return the closest point on segment AB to the given point.

        @param point: Query point as an array-like of length 3.
        @return: Closest point on the segment.
        @rtype: numpy.ndarray
        """
        ab = self.pointB - self.pointA
        t  = np.dot(point - self.pointA, ab) / np.dot(ab, ab)
        t  = np.clip(t, 0.0, 1.0)
        return self.pointA + t * ab

    def distanceToPoint(self, point) -> float:
        """
        Return the signed distance from a point to the capsule surface.

        Negative values indicate the point is inside the capsule.

        @param point: Query point as an array-like of length 3.
        @rtype: float
        """
        closest = self.closestPoint(point)
        return np.linalg.norm(point - closest) - self.radius

    @staticmethod
    def segmentSegmentDistance(A, B, C, D) -> float:
        """
        Return the minimum distance between segments AB and CD.

        @param A: Start of first segment.
        @param B: End of first segment.
        @param C: Start of second segment.
        @param D: End of second segment.
        @rtype: float
        """
        eps = 1e-8
        u = B - A
        v = D - C
        w = A - C

        a = np.dot(u, u)
        b = np.dot(u, v)
        c = np.dot(v, v)
        d = np.dot(u, w)
        e = np.dot(v, w)

        denom = a * c - b * b

        if denom < eps:
            s = 0.0
            t = e / c if c > eps else 0.0
        else:
            s = (b * e - c * d) / denom
            t = (a * e - b * d) / denom

        s = np.clip(s, 0.0, 1.0)
        t = np.clip(t, 0.0, 1.0)

        p = A + s * u
        q = C + t * v

        return np.linalg.norm(p - q)

    def intersects(self, other: "CapsuleCollider") -> bool:
        """
        Return True if this capsule overlaps with another capsule.

        @param other: The other capsule to test against.
        @type other: CapsuleCollider
        @rtype: bool
        """
        distance = self.segmentSegmentDistance(self.pointA, self.pointB, other.pointA, other.pointB)
        return distance <= (self.radius + other.radius)
