from __future__ import annotations
from reachyPart import ReachyPart
from capsuleCollider import CapsuleCollider
import config


class ReachyTorso(ReachyPart):

    def getCollision(self) -> list[CapsuleCollider]:
        return [CapsuleCollider(
            [0, 0, 0],
            [0, 0, -config.TORSO_SIZE],
            config.TORSO_COLLISION_RADIUS
        )]
