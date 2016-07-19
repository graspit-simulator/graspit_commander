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
    LoadWorld,
    ApproachToContact,
    ComputeQuality,
    DynamicAutoGraspComplete,
    ImportObstacle,
    FindInitialContact,
    ImportGraspableBody, 
    ImportObstacle,
    ImportRobot,
    SaveImage,
    SaveWorld,
    ToggleAllCollisions
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
    LoadWorldException,
    ImportException,
    SaveImageException,
    SaveWorldException,
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

    @staticmethod
    def approachToContact(moveDist=200, oneStep=False, id=0):
        _wait_for_service('approachToContact')

        serviceProxy = rospy.ServiceProxy('approachToContact', ApproachToContact)
        result = serviceProxy(moveDist, oneStep, id)

        if result.result is ApproachToContact._response_class.RESULT_SUCCESS:
            return
        elif result.result is ApproachToContact._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)

    @staticmethod
    def findInitialContact(id=0, moveDist=200):
        _wait_for_service('findInitialContact')

        serviceProxy = rospy.ServiceProxy('findInitialContact', FindInitialContact)
        result = serviceProxy(id, moveDist)

        if result.result is FindInitialContact._response_class.RESULT_SUCCESS:
            return
        elif result.result is FindInitialContact._response_class.RESULT_FAILURE:
            raise InvalidRobotIDException(id)

    @staticmethod
    def computeQuality(id=0):
        _wait_for_service('computeQuality')

        serviceProxy = rospy.ServiceProxy('computeQuality', ComputeQuality)
        result = serviceProxy(id)

        if result.result is ComputeQuality._response_class.RESULT_SUCCESS:
            return result
        elif result.result is ComputeQuality._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)
        elif result.result is ComputeQuality._response_class.RESULT_COLLISION:
            raise InvalidRobotPoseException()

    @staticmethod
    def dynamicAutoGraspComplete(id=0):
        _wait_for_service('dynamicAutoGraspComplete')

        serviceProxy = rospy.ServiceProxy('dynamicAutoGraspComplete', DynamicAutoGraspComplete)
        result = serviceProxy(id)

        if result.result is DynamicAutoGraspComplete._response_class.RESULT_SUCCESS:
            return result.GraspComplete
        elif result.result is DynamicAutoGraspComplete._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)

    @staticmethod
    def importObstacle(obstacleName):
        _wait_for_service('importObstacle')

        serviceProxy = rospy.ServiceProxy('importObstacle', ImportObstacle)
        result = serviceProxy(obstacleName)

        if result.result is ImportObstacle._response_class.RESULT_SUCCESS:
            return
        elif result.result is ImportObstacle._response_class.RESULT_FAILURE:
            raise ImportException("obstacle", obstacleName)

    @staticmethod
    def importGraspableBody(bodyName):
        _wait_for_service('importGraspableBody')

        serviceProxy = rospy.ServiceProxy('importGraspableBody', ImportGraspableBody)
        result = serviceProxy(bodyName)

        if result.result is ImportGraspableBody._response_class.RESULT_SUCCESS:
            return
        elif result.result is ImportGraspableBody._response_class.RESULT_FAILURE:
            raise ImportException("graspable body", bodyName)

    @staticmethod
    def importRobot(robotName):
        _wait_for_service('importRobot')

        serviceProxy = rospy.ServiceProxy('importRobot', ImportRobot)
        result = serviceProxy(robotName)

        if result.result is ImportRobot._response_class.RESULT_SUCCESS:
            return
        elif result.result is ImportRobot._response_class.RESULT_FAILURE:
            raise ImportException("robot", robotName)

    @staticmethod
    def saveImage(fileName):
        _wait_for_service('saveImage')

        serviceProxy = rospy.ServiceProxy('saveImage', SaveImage)
        result = serviceProxy(fileName)

        if result.result is SaveImage._response_class.RESULT_SUCCESS:
            return
        elif result.result is SaveImage._response_class.RESULT_FAILURE:
            raise SaveImageException()

    @staticmethod
    def saveWorld(fileName):
        _wait_for_service('saveWorld')

        serviceProxy = rospy.ServiceProxy('saveWorld', SaveWorld)
        result = serviceProxy(fileName)

        if result.result is SaveWorld._response_class.RESULT_SUCCESS:
            return
        elif result.result is SaveWorld._response_class.RESULT_FAILURE:
            raise SaveWorldException()

    @staticmethod
    def toggleAllCollisions(enableCollisions):
        _wait_for_service('toggleAllCollisions')

        serviceProxy = rospy.ServiceProxy('toggleAllCollisions', ToggleAllCollisions)
        serviceProxy(enableCollisions)
