import rospy

from graspit_interface.srv import GetRobot

class GraspitCommander(object): 
    """
    Python interface for interacting with GraspIt
    """

    @staticmethod
    def getRobots():
        pass

    @staticmethod
    def getRobot(id):
        rospy.wait_for_service('getRobot')
        
        serviceProxy = rospy.ServiceProxy('getRobot', GetRobot)
        robot = serviceProxy(id)

        return robot

    @staticmethod
    def setRobotPose(id, pose):
        pass

    @staticmethod
    def getGraspableBodies():
        pass

    @staticmethod
    def getGraspableBody(id):
        pass

    @staticmethod
    def setGraspableBodyPose(id, pose):
        pass

    @staticmethod
    def getObstacles():
        pass

    @staticmethod
    def getObstacle(id):
        pass

    @staticmethod
    def setObstaclePose(id, pose):
        pass

    @staticmethod
    def getDynamics():
        pass

    @staticmethod
    def setDynamics(enabled):
        pass

