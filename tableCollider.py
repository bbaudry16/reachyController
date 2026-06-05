from __future__ import annotations
import numpy as np


class TableCollider:
    """
    Table décrite par une boîte AABB (Axis-Aligned Bounding Box).

    Repère Reachy (origine = centre torse, hauteur épaules) :
        X = avant du robot
        Y = gauche du robot
        Z = haut

    Paramètres
    ----------
    x_min, x_max : profondeur (avant/arrière).  x_min < x_max.
    y_min, y_max : largeur (gauche/droite).      y_min < y_max.
    z_min, z_max : hauteur (bas/haut).           z_min < z_max.
                   z_max correspond au dessus de la table (surface),
                   z_min au dessous (sol ou châssis).

    Exemple typique — robot assis devant une table :
        x_min =  0.10   (bord avant de la table à 10 cm devant le torse)
        x_max =  0.80   (bord arrière à 80 cm)
        y_min = -0.50   (bord gauche)
        y_max =  0.50   (bord droit)
        z_min = -0.60   (dessous de la table)
        z_max = -0.40   (dessus de la table / surface)

    Un point P est en collision si :
        x_min ≤ P.x ≤ x_max  AND
        y_min ≤ P.y ≤ y_max  AND
        z_min ≤ P.z ≤ z_max

    Comportement physiquement correct :
        - Le bras peut descendre en DESSOUS de z_max si x < x_min
          (il dépasse de la table côté robot → pas de collision).
        - Le bras peut passer en dessous de z_max si y < y_min ou y > y_max
          (il dépasse latéralement).
        - Seul le volume intérieur de la boîte est interdit.
    """

    def __init__(self,
                 x_min: float, x_max: float,
                 y_min: float, y_max: float,
                 z_min: float, z_max: float) -> None:
        if x_min >= x_max or y_min >= y_max or z_min >= z_max:
            raise ValueError(
                "TableCollider: chaque dimension doit avoir min < max. "
                f"Reçu x=[{x_min},{x_max}] y=[{y_min},{y_max}] z=[{z_min},{z_max}]"
            )
        self.x_min = x_min;  self.x_max = x_max
        self.y_min = y_min;  self.y_max = y_max
        self.z_min = z_min;  self.z_max = z_max

    def containsPoint(self, point) -> bool:
        """Retourne True si le point est à l'intérieur du volume de la table."""
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        return (self.x_min <= x <= self.x_max and
                self.y_min <= y <= self.y_max and
                self.z_min <= z <= self.z_max)

    def distanceToPoint(self, point) -> float:
        """
        Distance signée d'un point à la boîte.
        Négatif = à l'intérieur, positif = à l'extérieur.
        """
        p = np.array([float(point[0]), float(point[1]), float(point[2])])
        lo = np.array([self.x_min, self.y_min, self.z_min])
        hi = np.array([self.x_max, self.y_max, self.z_max])
        # Distance extérieure (0 si dedans)
        d_outside = np.linalg.norm(np.maximum(0, np.maximum(lo - p, p - hi)))
        # Distance intérieure (0 si dehors)
        d_inside  = np.min(np.minimum(p - lo, hi - p))
        if d_outside > 0:
            return float(d_outside)   # dehors → positif
        return -float(d_inside)       # dedans → négatif

    def toDict(self) -> dict:
        return {
            "x_min": self.x_min, "x_max": self.x_max,
            "y_min": self.y_min, "y_max": self.y_max,
            "z_min": self.z_min, "z_max": self.z_max,
        }

    @classmethod
    def fromDict(cls, d: dict) -> "TableCollider":
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
        Constructeur pratique : définit la table par sa surface et son épaisseur.
        z_max = z_surface (dessus), z_min = z_surface - thickness.
        """
        return cls(x_min, x_max,
                   y_min, y_max,
                   z_surface - thickness, z_surface)

    def __repr__(self) -> str:
        return (f"TableCollider(x=[{self.x_min},{self.x_max}] "
                f"y=[{self.y_min},{self.y_max}] "
                f"z=[{self.z_min},{self.z_max}])")