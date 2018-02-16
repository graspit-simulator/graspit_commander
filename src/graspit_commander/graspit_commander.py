import rospy
import actionlib
from rospy.exceptions import ROSException
import geometry_msgs

from graspit_interface.msg import (
    Body,
    Energy,
    GraspableBody,
    Grasp,
    Planner,
    Robot,
    SearchContact,
    SearchSpace,
    SimAnnParams,
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
    ComputeEnergy,
    DynamicAutoGraspComplete,
    ImportObstacle,
    FindInitialContact,
    ImportGraspableBody, 
    ImportObstacle,
    ImportRobot,
    SaveImage,
    SaveWorld,
    ToggleAllCollisions,
    ForceRobotDOF,
    MoveDOFToContacts
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
    ImportRobotException,
    ImportBodyException,
    ImportObstacleException,
    ImportGraspableBodyException,
    SaveImageException,
    SaveWorldException,
    InvalidDynamicsModeException,
    InvalidEnergyTypeException
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
    GRASPIT_NODE_NAME = "/graspit/"

    @staticmethod
    def getRobots():
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'getRobots')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'getRobots', GetRobots)
        robotList = serviceProxy()

        return robotList

    @staticmethod
    def getRobot(id=0):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'getRobot')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'getRobot', GetRobot)
        robot = serviceProxy(id)

        if robot.result is GetRobot._response_class.RESULT_SUCCESS:
            return robot
        elif robot.result is GetRobot._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)

    @staticmethod
    def setRobotPose(pose, id=0):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'setRobotPose')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'setRobotPose', SetRobotPose)
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
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'getGraspableBodies')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'getGraspableBodies', GetGraspableBodies)
        graspableBodyList = serviceProxy()

        return graspableBodyList

    @staticmethod
    def getGraspableBody(id):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'getGraspableBody')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'getGraspableBody', GetGraspableBody)
        graspableBody = serviceProxy(id)

        if graspableBody.result is GetGraspableBody._response_class.RESULT_SUCCESS:
            return graspableBody
        elif graspableBody.result is GetGraspableBody._response_class.RESULT_INVALID_ID:
            raise InvalidGraspableBodyIDException(id)

    @staticmethod
    def setGraspableBodyPose(id, pose):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'setGraspableBodyPose')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'setGraspableBodyPose', SetGraspableBodyPose)
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
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'getBodies')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'getBodies', GetBodies)
        bodyList = serviceProxy()

        return bodyList

    @staticmethod
    def getBody(id):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'getBody')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'getBody', GetBody)
        body = serviceProxy(id)

        if body.result is GetBody._response_class.RESULT_SUCCESS:
            return body
        elif body.result is GetBody._response_class.RESULT_INVALID_ID:
            raise InvalidBodyIDException(id)

    @staticmethod
    def setBodyPose(id, pose):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'setBodyPose')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'setBodyPose', SetBodyPose)
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
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'getDynamics')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'getDynamics', GetDynamics)
        result = serviceProxy()

        return result.dynamicsEnabled


    @staticmethod
    def setDynamics(enabled):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'setDynamics')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'setDynamics', SetDynamics)
        serviceProxy(enabled)

    @staticmethod
    def autoGrasp(id=0):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'autoGrasp')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'autoGrasp', AutoGrasp)
        result = serviceProxy(id)

        if result.result is AutoGrasp._response_class.RESULT_SUCCESS:
            return
        elif result.result is AutoGrasp._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)

    @staticmethod
    def autoOpen(id=0):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'autoOpen')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'autoOpen', AutoOpen)
        result = serviceProxy(id)

        if result.result is AutoOpen._response_class.RESULT_SUCCESS:
            return
        elif result.result is AutoOpen._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)

    @staticmethod
    def forceRobotDof(dofs, id=0):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'forceRobotDof')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'forceRobotDof', ForceRobotDOF)
        result = serviceProxy(id, dofs)

        if result.result is ForceRobotDOF._response_class.RESULT_SUCCESS:
            return
        elif result.result is ForceRobotDOF._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)
        elif result.result is ForceRobotDOF._response_class.RESULT_DOF_OUT_OF_RANGE:
            raise InvalidRobotDOFOutOfRangeException()
        elif result.result is ForceRobotDOF._response_class.RESULT_DOF_COUNT_MISMATCH:
            raise InvalidRobotDOFCountMismatchException()
        elif result.result is ForceRobotDOF._response_class.RESULT_DYNAMICS_MODE_ENABLED:
            raise InvalidDynamicsModeException()

    @staticmethod
    def moveDOFToContacts(dofs, desired_steps, stop_at_contact, id=0):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'moveDOFToContacts')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'moveDOFToContacts', MoveDOFToContacts)
        result = serviceProxy(id, dofs, desired_steps, stop_at_contact)

        if result.result is MoveDOFToContacts._response_class.RESULT_SUCCESS:
            return
        elif result.result is MoveDOFToContacts._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)
        elif result.result is MoveDOFToContacts._response_class.RESULT_DOF_OUT_OF_RANGE:
            raise InvalidRobotDOFOutOfRangeException()
        elif result.result is MoveDOFToContacts._response_class.RESULT_DOF_COUNT_MISMATCH:
            raise InvalidRobotDOFCountMismatchException()
        elif result.result is MoveDOFToContacts._response_class.RESULT_DYNAMICS_MODE_ENABLED:
            raise InvalidDynamicsModeException()

    @staticmethod
    def setRobotDesiredDOF(dofs, dof_velocities, id=0):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'setRobotDesiredDOF')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'setRobotDesiredDOF', SetRobotDesiredDOF)
        result = serviceProxy(id, dofs, dof_velocities)

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
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'clearWorld')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'clearWorld', ClearWorld)
        result = serviceProxy()

        if result.result is ClearWorld._response_class.RESULT_SUCCESS:
            return
        elif result.result is ClearWorld._response_class.RESULT_FAILURE:
            raise ClearWorldException()

    @staticmethod
    def loadWorld(worldFile):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'loadWorld')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'loadWorld', LoadWorld)
        result = serviceProxy(worldFile)

        if result.result is LoadWorld._response_class.RESULT_SUCCESS:
            return
        elif result.result is LoadWorld._response_class.RESULT_FAILURE:
            raise LoadWorldException()

    @classmethod
    def planGrasps(cls,
                   graspable_body_id=0,
                   planner=Planner(Planner.SIM_ANN),
                   search_energy="GUIDED_POTENTIAL_QUALITY_ENERGY",
                   search_space=SearchSpace(SearchSpace.SPACE_AXIS_ANGLE),
                   search_contact=SearchContact(SearchContact.CONTACT_PRESET),
                   max_steps=70000,
                   feedback_cb=None,
                   feedback_num_steps=-1,
                   sim_ann_params=SimAnnParams()):
        try:
            rospy.init_node(cls.ROS_NODE_NAME, anonymous=True)
        except ROSException:
            pass

        client = actionlib.SimpleActionClient(GraspitCommander.GRASPIT_NODE_NAME + 'planGrasps', PlanGraspsAction)
        _wait_for_action_server(client)

        goal = PlanGraspsGoal(graspable_body_id=graspable_body_id,
                              planner=planner,
                              search_energy=search_energy,
                              search_space=search_space,
                              search_contact=search_contact,
                              max_steps=max_steps,
                              feedback_num_steps=feedback_num_steps,
                              sim_ann_params=sim_ann_params)

        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()

        return client.get_result()

    @staticmethod
    def approachToContact(moveDist=200, oneStep=False, id=0):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'approachToContact')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'approachToContact', ApproachToContact)
        result = serviceProxy(moveDist, oneStep, id)

        if result.result is ApproachToContact._response_class.RESULT_SUCCESS:
            return
        elif result.result is ApproachToContact._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)

    @staticmethod
    def findInitialContact(id=0, moveDist=200):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'findInitialContact')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'findInitialContact', FindInitialContact)
        result = serviceProxy(id, moveDist)

        if result.result is FindInitialContact._response_class.RESULT_SUCCESS:
            return
        elif result.result is FindInitialContact._response_class.RESULT_FAILURE:
            raise InvalidRobotIDException(id)

    @staticmethod
    def computeQuality(id=0):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'computeQuality')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'computeQuality', ComputeQuality)
        result = serviceProxy(id)

        if result.result is ComputeQuality._response_class.RESULT_SUCCESS:
            return result
        elif result.result is ComputeQuality._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)
        elif result.result is ComputeQuality._response_class.RESULT_COLLISION:
            raise InvalidRobotPoseException()

    @staticmethod
    def computeEnergy(energy_type, hand_id=0, body_id=0):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'computeEnergy')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'computeEnergy', ComputeEnergy)
        result = serviceProxy(hand_id, body_id, energy_type)

        if result.result is ComputeEnergy._response_class.RESULT_SUCCESS:
            return result
        elif result.result is ComputeEnergy._response_class.RESULT_INVALID_HAND_ID:
            raise InvalidRobotIDException(hand_id)
        elif result.result is ComputeEnergy._response_class.RESULT_INVALID_BODY_ID:
            raise InvalidGraspableBodyIDException(body_id)
        elif result.result is ComputeEnergy._response_class.RESULT_INVALID_ENERGY_TYPE:
            raise InvalidEnergyTypeException(energy_type)

    @staticmethod
    def dynamicAutoGraspComplete(id=0):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'dynamicAutoGraspComplete')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'dynamicAutoGraspComplete', DynamicAutoGraspComplete)
        result = serviceProxy(id)

        if result.result is DynamicAutoGraspComplete._response_class.RESULT_SUCCESS:
            return result.GraspComplete
        elif result.result is DynamicAutoGraspComplete._response_class.RESULT_INVALID_ID:
            raise InvalidRobotIDException(id)

    @staticmethod
    def importObstacle(obstacleName, pose=None):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'importObstacle')

        if not pose:
            pose = geometry_msgs.msg.Pose()
            pose.orientation.w = 1

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'importObstacle', ImportObstacle)
        result = serviceProxy(obstacleName, pose)

        if result.result is ImportObstacle._response_class.RESULT_SUCCESS:
            return
        elif result.result is ImportObstacle._response_class.RESULT_FAILURE:
            raise ImportObstacleException(name=obstacleName)

    @staticmethod
    def importGraspableBody(bodyName, pose=None):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'importGraspableBody')

        if not pose:
            pose = geometry_msgs.msg.Pose()
            pose.orientation.w = 1

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'importGraspableBody', ImportGraspableBody)
        result = serviceProxy(bodyName, pose)

        if result.result is ImportGraspableBody._response_class.RESULT_SUCCESS:
            return
        elif result.result is ImportGraspableBody._response_class.RESULT_FAILURE:
            raise ImportGraspableBodyException(name=bodyName)

    @staticmethod
    def importRobot(robotName, pose=None):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'importRobot')

        if not pose:
            pose = geometry_msgs.msg.Pose()
            pose.orientation.w = 1

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'importRobot', ImportRobot)
        result = serviceProxy(robotName, pose)

        if result.result is ImportRobot._response_class.RESULT_SUCCESS:
            return
        elif result.result is ImportRobot._response_class.RESULT_FAILURE:
            raise ImportRobotException(name=robotName)

    @staticmethod
    def saveImage(fileName):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'saveImage')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'saveImage', SaveImage)
        result = serviceProxy(fileName)

        if result.result is SaveImage._response_class.RESULT_SUCCESS:
            return
        elif result.result is SaveImage._response_class.RESULT_FAILURE:
            raise SaveImageException()

    @staticmethod
    def saveWorld(fileName):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'saveWorld')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'saveWorld', SaveWorld)
        result = serviceProxy(fileName)

        if result.result is SaveWorld._response_class.RESULT_SUCCESS:
            return
        elif result.result is SaveWorld._response_class.RESULT_FAILURE:
            raise SaveWorldException()

    @staticmethod
    def toggleAllCollisions(enableCollisions):
        _wait_for_service(GraspitCommander.GRASPIT_NODE_NAME + 'toggleAllCollisions')

        serviceProxy = rospy.ServiceProxy(GraspitCommander.GRASPIT_NODE_NAME + 'toggleAllCollisions', ToggleAllCollisions)
        serviceProxy(enableCollisions)
