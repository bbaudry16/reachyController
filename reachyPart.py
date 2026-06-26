from __future__ import annotations
from .capsuleCollider import CapsuleCollider


class ReachyPart:
    """
    Base class for all Reachy robot parts.

    Subclasses override L{getCollision} to expose their collision geometry.
    """

    def getCollision(self) -> list[CapsuleCollider]:
        """
        Return the list of capsule colliders representing this part.

        Intended to be overridden by subclasses.

        @rtype: list[CapsuleCollider]
        @return: Empty list by default.
        """
        return []
