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

        torso_colliders = self._torso.getCollision()

        # ── Arm vs opposite arm ───────────────────────────────────────────────
        # Both capsules (upper arm and forearm) checked against each other.
        for moving in moving_colliders:
            for static in static_colliders:
                if moving.intersects(static):
                    return False

        # ── Arm vs torso ──────────────────────────────────────────────────────
        # Only the FOREARM capsule (index 1 = midpoint→hand) is checked against
        # the torso. The upper arm capsule (shoulder→midpoint) uses a geometric
        # midpoint heuristic based on the FK segment shoulder→hand, which can
        # cross close to the torso axis even in safe poses (especially the right
        # arm whose shoulder is at X=-0.19m). Checking only the forearm avoids
        # false positives while still catching real arm-into-torso collisions.
        forearm_only = moving_colliders[1:]
        for moving in forearm_only:
            for static in torso_colliders:
                if moving.intersects(static):
                    return False

        return True