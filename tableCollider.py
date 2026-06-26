from __future__ import annotations
import numpy as np


class TableCollider:
    """
    Axis-aligned bounding box (AABB) representing a table obstacle.

    The Reachy coordinate frame has X+ forward, Y+ left, Z+ up, with the
    origin at the torso center at shoulder height.

    A point is in collision if it falls within the closed box defined by
    [x_min, x_max] x [y_min, y_max] x [z_min, z_max].

    @ivar x_min: Minimum X bound (backward direction).
    @type x_min: float
    @ivar x_max: Maximum X bound (forward direction).
    @type x_max: float
    @ivar y_min: Minimum Y bound (right direction).
    @type y_min: float
    @ivar y_max: Maximum Y bound (left direction).
    @type y_max: float
    @ivar z_min: Minimum Z bound (bottom of table).
    @type z_min: float
    @ivar z_max: Maximum Z bound (table surface).
    @type z_max: float
    """

    def __init__(self,
                 x_min: float, x_max: float,
                 y_min: float, y_max: float,
                 z_min: float, z_max: float) -> None:
        """
        @param x_min: Minimum X coordinate.
        @type x_min: float
        @param x_max: Maximum X coordinate.
        @type x_max: float
        @param y_min: Minimum Y coordinate.
        @type y_min: float
        @param y_max: Maximum Y coordinate.
        @type y_max: float
        @param z_min: Minimum Z coordinate.
        @type z_min: float
        @param z_max: Maximum Z coordinate.
        @type z_max: float
        @raise ValueError: If any dimension has min >= max.
        """
        if x_min >= x_max or y_min >= y_max or z_min >= z_max:
            raise ValueError(
                f"TableCollider: each dimension must have min < max. "
                f"Got x=[{x_min},{x_max}] y=[{y_min},{y_max}] z=[{z_min},{z_max}]"
            )
        self.x_min = x_min;  self.x_max = x_max
        self.y_min = y_min;  self.y_max = y_max
        self.z_min = z_min;  self.z_max = z_max

    def containsPoint(self, point) -> bool:
        """
        Return True if the given point is inside the table volume.

        @param point: 3D point as an array-like of length 3.
        @rtype: bool
        """
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        return (self.x_min <= x <= self.x_max and
                self.y_min <= y <= self.y_max and
                self.z_min <= z <= self.z_max)

    def distanceToPoint(self, point) -> float:
        """
        Return the signed distance from a point to the box surface.

        Positive means outside; negative means inside.

        @param point: 3D point as an array-like of length 3.
        @rtype: float
        """
        p  = np.array([float(point[0]), float(point[1]), float(point[2])])
        lo = np.array([self.x_min, self.y_min, self.z_min])
        hi = np.array([self.x_max, self.y_max, self.z_max])
        d_outside = np.linalg.norm(np.maximum(0, np.maximum(lo - p, p - hi)))
        d_inside  = np.min(np.minimum(p - lo, hi - p))
        if d_outside > 0:
            return float(d_outside)
        return -float(d_inside)

    def toDict(self) -> dict:
        """
        Serialize the collider to a plain dictionary.

        @rtype: dict
        """
        return {
            "x_min": self.x_min, "x_max": self.x_max,
            "y_min": self.y_min, "y_max": self.y_max,
            "z_min": self.z_min, "z_max": self.z_max,
        }

    @classmethod
    def fromDict(cls, d: dict) -> "TableCollider":
        """
        Deserialize a collider from a plain dictionary.

        @param d: Dictionary with keys x_min, x_max, y_min, y_max, z_min, z_max.
        @type d: dict
        @rtype: TableCollider
        """
        return cls(d["x_min"], d["x_max"],
                   d["y_min"], d["y_max"],
                   d["z_min"], d["z_max"])

    @classmethod
    def fromSurface(cls,
                    x_min: float, x_max: float,
                    y_min: float, y_max: float,
                    z_surface: float,
                    thickness: float = 0.10) -> "TableCollider":
        """
        Construct a collider from the table surface height and thickness.

        z_max is set to z_surface and z_min to z_surface - thickness.

        @param x_min: Minimum X bound.
        @type x_min: float
        @param x_max: Maximum X bound.
        @type x_max: float
        @param y_min: Minimum Y bound.
        @type y_min: float
        @param y_max: Maximum Y bound.
        @type y_max: float
        @param z_surface: Z coordinate of the table surface (top face).
        @type z_surface: float
        @param thickness: Table thickness in meters (default 0.10).
        @type thickness: float
        @rtype: TableCollider
        """
        return cls(x_min, x_max,
                   y_min, y_max,
                   z_surface - thickness, z_surface)

    def __repr__(self) -> str:
        return (f"TableCollider(x=[{self.x_min},{self.x_max}] "
                f"y=[{self.y_min},{self.y_max}] "
                f"z=[{self.z_min},{self.z_max}])")
