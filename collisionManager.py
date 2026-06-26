from __future__ import annotations
from .capsuleCollider import CapsuleCollider
from .reachyPart import ReachyPart
from . import armController as ac
from . import consoleManager as cm


class CollisionManager:
    """
    Validates planned arm movements against collision constraints.

    Use L{askValidMovement} when only one arm is moving (reads the other arm
    live). Use L{askValidMovementBoth} for parallel moves, which checks both
    arms against their predicted future positions.

    @ivar _armRight: Right arm controller.
    @ivar _armLeft: Left arm controller.
    @ivar _arms: Mapping of arm ID to arm controller.
    @ivar _torso: Torso part for collision geometry.
    """

    def __init__(self, armRight: ac.ReachyArm, armLeft: ac.ReachyArm, torso: ReachyPart):
        """
        @param armRight: Right arm controller.
        @type armRight: ac.ReachyArm
        @param armLeft: Left arm controller.
        @type armLeft: ac.ReachyArm
        @param torso: Torso part providing collision capsules.
        @type torso: ReachyPart
        """
        self._armRight = armRight
        self._armLeft  = armLeft
        self._arms     = {armRight.getArmId(): armRight, armLeft.getArmId(): armLeft}
        self._torso    = torso

    def askValidMovement(self, armId: str, joint_dict: dict) -> bool:
        """
        Check a single arm movement against the live position of the other arm.

        Use this when only one arm is moving.

        @param armId: ID of the moving arm.
        @type armId: str
        @param joint_dict: Target joint positions for the moving arm.
        @type joint_dict: dict
        @rtype: bool
        @return: True if no collision is detected.
        """
        moving_arm  = self._arms[armId]
        static_arm  = self._armRight if armId == self._armLeft.getArmId() else self._armLeft

        moving_colliders = moving_arm.getCollisionFromPosition(joint_dict)
        static_colliders = static_arm.getCollision()
        torso_colliders  = self._torso.getCollision()

        return self._check(moving_colliders, static_colliders, torso_colliders)

    def askValidMovementBoth(self,
                              joint_dict_right: dict | None,
                              joint_dict_left:  dict | None) -> tuple[bool, bool]:
        """
        Check both arms simultaneously using their predicted target positions.

        Pass None for an arm to use its current live position instead.

        @param joint_dict_right: Predicted target for the right arm, or None.
        @type joint_dict_right: dict or None
        @param joint_dict_left: Predicted target for the left arm, or None.
        @type joint_dict_left: dict or None
        @rtype: tuple[bool, bool]
        @return: (right_ok, left_ok) — independent validity per arm.
        """
        if joint_dict_right is not None:
            caps_right = self._armRight.getCollisionFromPosition(joint_dict_right)
        else:
            caps_right = self._armRight.getCollision()

        if joint_dict_left is not None:
            caps_left = self._armLeft.getCollisionFromPosition(joint_dict_left)
        else:
            caps_left = self._armLeft.getCollision()

        torso_colliders = self._torso.getCollision()

        right_ok = self._check(caps_right, caps_left,  torso_colliders)
        left_ok  = self._check(caps_left,  caps_right, torso_colliders)

        return right_ok, left_ok

    @staticmethod
    def _check(moving_colliders: list[CapsuleCollider],
               static_colliders: list[CapsuleCollider],
               torso_colliders:  list[CapsuleCollider]) -> bool:
        """
        Return True if no collision is detected.

        Checks moving arm against static arm (all capsules) and against torso
        (forearm and hand only — the upper arm capsule is excluded to avoid
        false positives when the arm hangs at rest near the torso axis).

        @param moving_colliders: Capsules of the arm being checked.
        @type moving_colliders: list[CapsuleCollider]
        @param static_colliders: Capsules of the other arm.
        @type static_colliders: list[CapsuleCollider]
        @param torso_colliders: Capsules of the torso.
        @type torso_colliders: list[CapsuleCollider]
        @rtype: bool
        """
        for moving in moving_colliders:
            for static in static_colliders:
                if moving.intersects(static):
                    cm.MKprintSafety("Arm-arm collision detected", "CollisionManager", cm.Color.RED)
                    return False

        for moving in moving_colliders[1:]:
            for static in torso_colliders:
                if moving.intersects(static):
                    cm.MKprintSafety("Arm-torso collision detected", "CollisionManager", cm.Color.RED)
                    return False

        return True
