import trimesh
from trimesh.collision import CollisionManager
from faser_math import tm
from faser_robot_kinematics import Arm, SP

class ColliderManager:
    def __init__(self):
        self.collision_objects = []

    def update(self):
        pass

    def checkCollisions(self):
        pass

class ColliderObject:
    def __init__(self, name):
        self.name = name

    def update(self):
        pass

    def checkExternalCollisions(self):
        pass

    def checkInternalCollisions(self):
        pass

    def checkAllCollisions(self):
        pass

class ColliderArm(ColliderObject):
    def __init__(self):
        self.super().__init__()

class ColliderSP(ColliderObject):
    def __init__(self):
        self.super().__init__()

class ColliderObstacle(ColliderObject):
    def __init__(self):
        self.super().__init__()


class Collider:

    def __init__(self):
        self.robots = {}
        self.obstacles = CollisionManager()

    def addBox(self, props, name, manager):
        pass

    def addCylinder(self, props, name, manager):
        pass

    def addSphere(self, props, name, manager):
        pass

    def addMesh(self, props, name, manager):
        pass

    def updateSerialArm(self, arm_name):
        pass

    def populateSerialArm(self, robot, manager):
        props = robot.col_props
        if props == None:
            props = robot.vis_props
        for i in range(len(props)):
            link_name = robot.link_names[i]
            p = props[i]
            if p[0] == 'box':
                self.addBox(p, link_name, manager)
            elif p[0] == 'cyl':
                self.addCylinder(p, link_name, manager)
            elif p[0] == 'spr':
                self.addSphere(p, link_name, manager)
            elif p[0] == 'msh':
                self.addMesh(p, link_name, manager)

    def addRobot(self, robot, robot_name):
        if isinstance(robot, Arm):
            new_manager = CollisionManager()
            self.populateSerialArm(robot, new_manager)
            self.robots[robot_name] = new_manager
