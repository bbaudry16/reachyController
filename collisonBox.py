import numpy as np
import armController
import torsoController

class CapsuleCollider:
    def __init__(self, pointA, pointB, radius: float):
        self.pointA = np.array(pointA, dtype=float)
        self.pointB = np.array(pointB, dtype=float)
        self.radius = radius


    def closestPoint(self, point):
        """
        Closest point on the segment AB to a point
        """
        ab = self.pointB - self.pointA
        t = np.dot(point - self.pointA, ab) / np.dot(ab, ab)
        t = np.clip(t, 0.0, 1.0)
        return self.pointA + t * ab


    def distanceToPoint(self, point):
        """
        Signed distance from a point to the capsule surface
        """
        closest = self.closestPoint(point)
        return np.linalg.norm(point - closest) - self.radius


    @staticmethod
    def segmentSegmentDistance(A, B, C, D):
        """
        Minimum distance between segments AB and CD
        """
        eps = 1e-8

        u = B - A
        v = D - C
        w = A - C

        a = np.dot(u, u)
        b = np.dot(u, v)
        c = np.dot(v, v)
        d = np.dot(u, w)
        e = np.dot(v, w)

        denom = a * c - b * b

        if denom < eps:
            s = 0.0
            t = e / c if c > eps else 0.0
        else:
            s = (b * e - c * d) / denom
            t = (a * e - b * d) / denom

        s = np.clip(s, 0.0, 1.0)
        t = np.clip(t, 0.0, 1.0)

        p = A + s * u
        q = C + t * v

        return np.linalg.norm(p - q)

    def intersects(self, other: "CapsuleCollider") -> bool:
        distance = self.segmentSegmentDistance(
            self.pointA, self.pointB,
            other.pointA, other.pointB
        )
        return distance <= (self.radius + other.radius)

class collisionSkeleton():

    def __init__(self, armRight : "armController" = None, armLeft : "armController" = None, torso : "torsoController.ReachyTorso" = None):
        if armRight == None and armLeft == None and torso == None:
            return
        
        self.reachyArm = {armRight.getArmId() : armRight, armLeft.getArmId() : armLeft}
        self.torso = torso

    def askValidMovement(self, armId : str, joint_dict : dict) -> bool:
        collider : list = []
        currentObjectCollider : list = []

        #get arm collider
        for arm in self.reachyArm.keys():
            if arm != armId:
                collider += self.reachyArm[arm].getCollision()
            else:
                currentObjectCollider += self.reachyArm[arm].getCollisionFromPosition(joint_dict)
        
        #get torso collider
        collider += self.torso.getCollision()

        ok : bool = True

        for currentCollider in currentObjectCollider:
            for colliderToTest in collider:
                ok = not currentCollider.intersects(colliderToTest)

                if not ok:
                    break
        
            if not ok:
                break
        
        return ok