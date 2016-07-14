import rospy
import actionlib
from rospy.exceptions import ROSException

from graspit_interface.msg import (
    Body,
    Energy,
    GraspableBody,
    Grasp,
    Planner,
    Robot,
    SearchContact,
    SearchEnergy,
    SearchSpace,
    PlanGraspsAction,
    PlanGraspsGoal
)
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
    SetDynamics,
    AutoGrasp,
    AutoOpen,
    SetRobotDesiredDOF,
    ClearWorld,
    LoadWorld
)

from graspit_exceptions import (
    GraspitTimeoutException,
    InvalidRobotIDException,
    InvalidRobotPoseException,
    RobotCollisionException,
    InvalidRobotDOFOutOfRangeException,
    InvalidRobotDOFCountMismatchException,
    InvalidGraspableBodyIDException,
    InvalidGraspableBodyPoseException,
    GraspableBodyCollisionException,
    InvalidBodyIDException,
    InvalidBodyPoseException,
    BodyCollisionException,
    ClearWorldException,
    LoadWorldException
)


def _wait_for_service(serviceName, timeout=1):
    try:
        rospy.wait_for_service(serviceName, timeout=timeout)
    except ROSException, e:
        raise GraspitTimeoutException(serviceName)

def _wait_for_action_server(client, timeout=1):
    try:
        client.wait_for_server(timeout=rospy.Duration(timeout))
    except ROSException, e:
        raise GraspitTimeoutException()


class GraspitCommander(object): 
    """
    Python interface for interacting with GraspIt
    """

    ROS_NODE_NAME = "GraspItCommanderNode"

    @staticmethod
    def getRobots():
        _wait_for_service('getRobots')

        serviceProxy = rospy.ServiceProxy('getRobots', GetRobots)
        robotList = serviceProxy()

        return robotList

    @staticmethod
    def getRobot(id=0):
        _wait_for_service('getRobot')
        
        serviceProxy = rospy.ServiceProxy('getRobot', GetRobot)
        robot = serviceProxy(id)

        if robot.result is GetRobot._response_class.RESULT_SUCCESS:
            return robot
        elif robot.result is GetRobot._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)

    @staticmethod
    def setRobotPose(pose, id=0):
        _wait_for_service('setRobotPose')

        serviceProxy = rospy.ServiceProxy('setRobotPose', SetRobotPose)
        result = serviceProxy(id, pose)

        if result.result is SetRobotPose._response_class.RESULT_SUCCESS:
            return
        elif result.result is SetRobotPose._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)
        elif result.result is SetRobotPose._response_class.RESULT_INVALID_POSE:
            raise InvalidRobotPoseException()
        elif result.result is SetRobotPose._response_class.RESULT_ROBOT_IN_COLLISION:
            raise RobotCollisionException()

    @staticmethod
    def getGraspableBodies():
        _wait_for_service('getGraspableBodies')

        serviceProxy = rospy.ServiceProxy('getGraspableBodies', GetGraspableBodies)
        graspableBodyList = serviceProxy()

        return graspableBodyList

    @staticmethod
    def getGraspableBody(id):
        _wait_for_service('getGraspableBody')

        serviceProxy = rospy.ServiceProxy('getGraspableBody', GetGraspableBody)
        graspableBody = serviceProxy(id)

        if graspableBody.result is GetGraspableBody._response_class.RESULT_SUCCESS:
            return graspableBody
        elif graspableBody.result is GetGraspableBody._response_class.RESULT_INVALID_ID:
            raise InvalidGraspableBodyIDException(id)

    @staticmethod
    def setGraspableBodyPose(id, pose):
        _wait_for_service('setGraspableBodyPose')

        serviceProxy = rospy.ServiceProxy('setGraspableBodyPose', SetGraspableBodyPose)
        result = serviceProxy(id, pose)

        if result.result is SetGraspableBodyPose._response_class.RESULT_SUCCESS:
            return
        elif result.result is SetGraspableBodyPose._response_class.RESULT_INVALID_ID:
            raise InvalidGraspableBodyIDException(id)
        elif result.result is SetGraspableBodyPose._response_class.RESULT_INVALID_POSE:
            raise InvalidGraspableBodyPoseException()
        elif result.result is SetGraspableBodyPose._response_class.RESULT_ROBOT_IN_COLLISION:
            raise GraspableBodyCollisionException()

    @staticmethod
    def getBodies():
        _wait_for_service('getBodies')

        serviceProxy = rospy.ServiceProxy('getBodies', GetBodies)
        bodyList = serviceProxy()

        return bodyList

    @staticmethod
    def getBody(id):
        _wait_for_service('getBody')

        serviceProxy = rospy.ServiceProxy('getBody', GetBody)
        body = serviceProxy(id)

        if body.result is GetBody._response_class.RESULT_SUCCESS:
            return body
        elif body.result is GetBody._response_class.RESULT_INVALID_ID:
            raise InvalidBodyIDException(id)

    @staticmethod
    def setBodyPose(id, pose):
        _wait_for_service('setBodyPose')

        serviceProxy = rospy.ServiceProxy('setBodyPose', SetBodyPose)
        result = serviceProxy(id, pose)

        if result.result is SetBodyPose._response_class.RESULT_SUCCESS:
            return
        elif result.result is SetBodyPose._response_class.RESULT_INVALID_ID:
            raise InvalidBodyIDException(id)
        elif result.result is SetBodyPose._response_class.RESULT_INVALID_POSE:
            raise InvalidBodyPoseException()
        elif result.result is SetBodyPose._response_class.RESULT_ROBOT_IN_COLLISION:
            raise BodyCollisionException()

    @staticmethod
    def getDynamics():
        _wait_for_service('getDynamics')

        serviceProxy = rospy.ServiceProxy('getDynamics', GetDynamics)
        result = serviceProxy()

        return result.dynamicsEnabled


    @staticmethod
    def setDynamics(enabled):
        _wait_for_service('setDynamics')

        serviceProxy = rospy.ServiceProxy('setDynamics', SetDynamics)
        serviceProxy(enabled)

    @staticmethod
    def autoGrasp(id=0):
        _wait_for_service('autoGrasp')

        serviceProxy = rospy.ServiceProxy('autoGrasp', AutoGrasp)
        result = serviceProxy(id)

        if result.result is AutoGrasp._response_class.RESULT_SUCCESS:
            return
        elif result.result is AutoGrasp._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)

    @staticmethod
    def autoOpen(id=0):
        _wait_for_service('autoOpen')

        serviceProxy = rospy.ServiceProxy('autoOpen', AutoOpen)
        result = serviceProxy(id)

        if result.result is AutoOpen._response_class.RESULT_SUCCESS:
            return
        elif result.result is AutoOpen._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)

    @staticmethod
    def setRobotDesiredDOF(dofs, id=0):
        _wait_for_service('setRobotDesiredDOF')

        serviceProxy = rospy.ServiceProxy('setRobotDesiredDOF', SetRobotDesiredDOF)
        result = serviceProxy(id, dofs)

        if result.result is SetRobotDesiredDOF._response_class.RESULT_SUCCESS:
            return
        elif result.result is SetRobotDesiredDOF._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)
        elif result.result is SetRobotDesiredDOF._response_class.RESULT_DOF_OUT_OF_RANGE:
            raise InvalidRobotDOFOutOfRangeException()
        elif result.result is SetRobotDesiredDOF._response_class.RESULT_DOF_COUNT_MISMATCH:
            raise InvalidRobotDOFCountMismatchException()

    @staticmethod
    def clearWorld():
        _wait_for_service('clearWorld')

        serviceProxy = rospy.ServiceProxy('clearWorld', ClearWorld)
        result = serviceProxy()

        if result.result is ClearWorld._response_class.RESULT_SUCCESS:
            return
        elif result.result is ClearWorld._response_class.RESULT_FAILURE:
            raise ClearWorldException()

    @staticmethod
    def loadWorld(worldFile):
        _wait_for_service('loadWorld')

        serviceProxy = rospy.ServiceProxy('loadWorld', LoadWorld)
        result = serviceProxy(worldFile)

        if result.result is LoadWorld._response_class.RESULT_SUCCESS:
            return
        elif result.result is LoadWorld._response_class.RESULT_FAILURE:
            raise LoadWorldException()

    @classmethod
    def planGrasps(cls,
                   graspable_body_id=0,
                   planner=Planner(Planner.SIM_ANN),
                   search_energy=SearchEnergy(SearchEnergy.ENERGY_CONTACT_QUALITY),
                   search_space=SearchSpace(SearchSpace.SPACE_AXIS_ANGLE),
                   search_contact=SearchContact(SearchContact.CONTACT_PRESET),
                   max_steps=70000):
        try:
            rospy.init_node(cls.ROS_NODE_NAME, anonymous=True)
        except ROSException:
            pass

        client = actionlib.SimpleActionClient('planGrasps', PlanGraspsAction)
        _wait_for_action_server(client)

        goal = PlanGraspsGoal(graspable_body_id=graspable_body_id,
                              planner=planner,
                              search_energy=search_energy,
                              search_space=search_space,
                              search_contact=search_contact,
                              max_steps=max_steps)

        client.send_goal_and_wait(goal)

        return client.get_result()
