from __future__ import annotations
from .capsuleCollider import CapsuleCollider
from .reachyPart import ReachyPart
from . import armController as ac
from . import consoleManager as cm


class CollisionManager:
    """
    Validates whether a planned movement is collision-free.

    askValidMovement(armId, joint_dict) — vérifie un seul bras contre
        la position LIVE de l'autre bras (usage hors parallel).

    askValidMovementBoth(joint_dict_right, joint_dict_left) — vérifie les
        deux bras simultanément en utilisant leurs cibles prédites mutuelles.
        C'est la méthode correcte pour un mouvement parallel, où les deux
        bras bougent en même temps et doivent être vérifiés l'un contre l'autre
        dans leur état FUTUR, pas leur état live.
    """

    def __init__(self, armRight: ac.ReachyArm, armLeft: ac.ReachyArm, torso: ReachyPart):
        self._armRight = armRight
        self._armLeft  = armLeft
        self._arms  = {armRight.getArmId(): armRight, armLeft.getArmId(): armLeft}
        self._torso = torso

    # ── Vérification d'un seul bras (bras opposé lu en live) ──────────────────
    def askValidMovement(self, armId: str, joint_dict: dict) -> bool:
        """
        Vérifie le mouvement d'un seul bras contre la position live de l'autre.
        Utilisé quand un seul bras bouge (pas de parallel).
        """
        moving_arm  = self._arms[armId]
        static_arm  = self._armRight if armId == self._armLeft.getArmId() else self._armLeft

        moving_colliders = moving_arm.getCollisionFromPosition(joint_dict)
        static_colliders = static_arm.getCollision()   # position live
        torso_colliders  = self._torso.getCollision()

        return self._check(moving_colliders, static_colliders, torso_colliders)

    # ── Vérification des deux bras simultanément (pour parallel) ─────────────
    def askValidMovementBoth(self,
                              joint_dict_right: dict | None,
                              joint_dict_left:  dict | None) -> tuple[bool, bool]:
        """
        Vérifie les deux bras contre leurs cibles prédites mutuelles.

        Paramètres
        ----------
        joint_dict_right : cible prédite du bras droit   (None = garder live)
        joint_dict_left  : cible prédite du bras gauche  (None = garder live)

        Retour
        ------
        (right_ok, left_ok) : booléens indépendants par bras.
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

        right_ok = self._check(caps_right, caps_left,   torso_colliders)
        left_ok  = self._check(caps_left,  caps_right,  torso_colliders)

        return right_ok, left_ok

    # ── Logique de check commune ───────────────────────────────────────────────
    @staticmethod
    def _check(moving_colliders: list[CapsuleCollider],
               static_colliders: list[CapsuleCollider],
               torso_colliders:  list[CapsuleCollider]) -> bool:
        """
        Retourne True si aucune collision n'est détectée.
        - Bras mobile vs bras statique : toutes les capsules.
        - Bras mobile vs torse : seulement avant-bras (cap1) et main (cap2).
          Le bras supérieur (cap0) est exclu pour éviter les faux positifs en
          pose neutre (le segment épaule→coude passe près de l'axe du torse).
        """
        for moving in moving_colliders:
            for static in static_colliders:
                if moving.intersects(static):
                    cm.MKprintSafety(
                        "Arm-arm collision detected",
                        "CollisionManager", cm.Color.RED
                    )
                    return False

        for moving in moving_colliders[1:]:   # cap1 et cap2 seulement
            for static in torso_colliders:
                if moving.intersects(static):
                    cm.MKprintSafety(
                        "Arm-torso collision detected",
                        "CollisionManager", cm.Color.RED
                    )
                    return False

        return True