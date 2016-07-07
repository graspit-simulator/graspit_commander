import rospy

from graspit_interface.srv import (
    GetRobots,
    GetRobot,
    SetRobotPose,
    GetGraspableBodies,
    GetGraspableBody,
    SetGraspableBodyPose,
    GetBodies,
    GetBody,
    SetBodyPose,
    GetDynamics,
    SetDynamics
)

from graspit_exceptions import (
    InvalidRobotIDException,
    InvalidRobotPoseException,
    RobotCollisionException,
    InvalidGraspableBodyIDException,
    InvalidGraspableBodyPoseException,
    GraspableBodyCollisionException,
    InvalidBodyIDException,
    InvalidBodyPoseException,
    BodyCollisionException
)

class GraspitCommander(object): 
    """
    Python interface for interacting with GraspIt
    """

    @staticmethod
    def getRobots():
        rospy.wait_for_service('getRobots')

        serviceProxy = rospy.ServiceProxy('getRobots', GetRobots)
        robotList = serviceProxy()

        return robotList

    @staticmethod
    def getRobot(id):
        rospy.wait_for_service('getRobot')
        
        serviceProxy = rospy.ServiceProxy('getRobot', GetRobot)
        robot = serviceProxy(id)

        if robot.result is GetRobot._response_class.RESULT_SUCCESS:
            return robot
        elif robot.result is GetRobot._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)

    @staticmethod
    def setRobotPose(id, pose):
        rospy.wait_for_service('setRobotPose')

        serviceProxy = rospy.ServiceProxy('setRobotPose', SetRobotPose)
        result = serviceProxy(id, pose)

        if result.result is SetRobotPose._response_class.RESULT_SUCCESS:
            return True
        elif result.result is SetRobotPose._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)
        elif result.result is SetRobotPose._response_class.RESULT_INVALID_POSE:
            raise InvalidRobotPoseException()
        elif result.result is SetRobotPose._response_class.RESULT_ROBOT_IN_COLLISION:
            raise RobotCollisionException()

    @staticmethod
    def getGraspableBodies():
        rospy.wait_for_service('getGraspableBodies')

        serviceProxy = rospy.ServiceProxy('getGraspableBodies', GetGraspableBodies)
        graspableBodyList = serviceProxy()

        return graspableBodyList

    @staticmethod
    def getGraspableBody(id):
        rospy.wait_for_service('getGraspableBody')

        serviceProxy = rospy.ServiceProxy('getGraspableBody', GetGraspableBody)
        graspableBody = serviceProxy(id)

        if graspableBody.result is GetGraspableBody._response_class.RESULT_SUCCESS:
            return graspableBody
        elif graspableBody.result is GetGraspableBody._response_class.RESULT_INVALID_ID:
            raise InvalidGraspableBodyIDException(id)

    @staticmethod
    def setGraspableBodyPose(id, pose):
        rospy.wait_for_service('setGraspableBodyPose')

        serviceProxy = rospy.ServiceProxy('setGraspableBodyPose', SetGraspableBodyPose)
        result = serviceProxy(id, pose)

        if result.result is SetGraspableBodyPose._response_class.RESULT_SUCCESS:
            return True
        elif result.result is SetGraspableBodyPose._response_class.RESULT_INVALID_ID:
            raise InvalidGraspableBodyIDException(id)
        elif result.result is SetGraspableBodyPose._response_class.RESULT_INVALID_POSE:
            raise InvalidGraspableBodyPoseException()
        elif result.result is SetGraspableBodyPose._response_class.RESULT_ROBOT_IN_COLLISION:
            raise GraspableBodyCollisionException()

    @staticmethod
    def getBodies():
        rospy.wait_for_service('getBodies')

        serviceProxy = rospy.ServiceProxy('getBodies', GetBodies)
        bodyList = serviceProxy()

        return bodyList

    @staticmethod
    def getBody(id):
        rospy.wait_for_service('getBody')

        serviceProxy = rospy.ServiceProxy('getBody', GetBody)
        body = serviceProxy(id)

        if body.result is GetBody._response_class.RESULT_SUCCESS:
            return body
        elif body.result is GetBody._response_class.RESULT_INVALID_ID:
            raise InvalidBodyIDException(id)

    @staticmethod
    def setBodyPose(id, pose):
        rospy.wait_for_service('setBodyPose')

        serviceProxy = rospy.ServiceProxy('setBodyPose', SetBodyPose)
        result = serviceProxy(id, pose)

        if result.result is SetBodyPose._response_class.RESULT_SUCCESS:
            return True
        elif result.result is SetBodyPose._response_class.RESULT_INVALID_ID:
            raise InvalidBodyIDException(id)
        elif result.result is SetBodyPose._response_class.RESULT_INVALID_POSE:
            raise InvalidBodyPoseException()
        elif result.result is SetBodyPose._response_class.RESULT_ROBOT_IN_COLLISION:
            raise BodyCollisionException()

    @staticmethod
    def getDynamics():
        rospy.wait_for_service('getDynamics')

        serviceProxy = rospy.ServiceProxy('getDynamics', GetDynamics)
        result = serviceProxy()

        return result.dynamicsEnabled


    @staticmethod
    def setDynamics(enabled):
        rospy.wait_for_service('getRobot')

        serviceProxy = rospy.ServiceProxy('getRobot', GetRobot)
        serviceProxy(enabled)
