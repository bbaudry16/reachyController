from __future__ import annotations
from .reachyPart import ReachyPart
from .capsuleCollider import CapsuleCollider
from . import config


class ReachyTorso(ReachyPart):
    """
    Collision representation of the Reachy torso.

    Models the torso as a single vertical capsule extending downward from
    the shoulder-height origin.
    """

    def getCollision(self) -> list[CapsuleCollider]:
        """
        Return a single capsule covering the torso volume.

        @rtype: list[CapsuleCollider]
        """
        return [
            CapsuleCollider([0, 0, 0], [0, 0, -config.TORSO_SIZE], config.TORSO_COLLISION_RADIUS),
        ]
