from __future__ import annotations
from .capsuleCollider import CapsuleCollider
from .reachyPart import ReachyPart
from . import armController as ac
from . import consoleManager as cm


class CollisionManager:
    """
    Holds references to all robot parts and validates whether
    a planned movement for one arm is collision-free against
    all other parts.
    """

    def __init__(self, armRight: ac.ReachyArm, armLeft: ac.ReachyArm, torso: ReachyPart):
        self._arms  = {armRight.getArmId(): armRight, armLeft.getArmId(): armLeft}
        self._torso = torso

    def askValidMovement(self, armId: str, joint_dict: dict) -> bool:
        static_colliders  : list[CapsuleCollider] = []
        moving_colliders  : list[CapsuleCollider] = []

        for arm_id, arm in self._arms.items():
            if arm_id != armId:
                static_colliders += arm.getCollision()
            else:
                moving_colliders += arm.getCollisionFromPosition(joint_dict)

        static_colliders += self._torso.getCollision()

        for moving in moving_colliders:
            for static in static_colliders:
                if moving.intersects(static):
                    dist = CapsuleCollider.segmentSegmentDistance(
                        moving.pointA, moving.pointB,
                        static.pointA, static.pointB
                    )
                    cm.MKprintDebug(
                        f"Collision! moving={moving.pointA.tolist()}->{moving.pointB.tolist()} r={moving.radius} | "
                        f"static={static.pointA.tolist()}->{static.pointB.tolist()} r={static.radius} | "
                        f"dist={dist:.4f} sum_r={moving.radius + static.radius:.4f}",
                        "CollisionManager", cm.Color.RED
                    )
                    return False

        return True
