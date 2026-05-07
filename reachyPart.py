from __future__ import annotations
from .capsuleCollider import CapsuleCollider


class ReachyPart:

    def getCollision(self) -> list[CapsuleCollider]:
        """
        meant to be overwritten, return a list of collider box
        """
        return []
