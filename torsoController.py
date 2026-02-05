import reachyPart as rp
import collisonBox as cb


class ReachyTorso(rp.ReachyPart):

    TORSO_SIZE : float = 0.263

    def getCollision(self):
        return [cb.CapsuleCollider([0, 0, 0], [0, 0, -self.TORSO_SIZE], 0.02)]