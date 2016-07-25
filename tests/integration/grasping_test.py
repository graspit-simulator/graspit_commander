import time
import unittest

from graspit_commander.graspit_commander import GraspitCommander


class GraspingTest(unittest.TestCase):

    ROS_SERVICE_TIMEOUT = 1

    """Make sure GraspIt! has a clean world and we can connect to it"""
    def setUp(self):
        GraspitCommander.clearWorld()

    @unittest.skip("Results not idempotent across different machines")
    def testStaticGraspExecution(self):
        GraspitCommander.loadWorld("plannerMug")
        GraspitCommander.approachToContact()
        GraspitCommander.autoGrasp()
        result = GraspitCommander.computeQuality()

        self.assertAlmostEqual(result.volume, 0.004336969, 4)
        self.assertAlmostEqual(result.epsilon, 0.046997464, 4)

    @unittest.skip("Results not idempotent across different machines")
    def testDynamicGraspExecution(self):
        GraspitCommander.loadWorld("plannerMug")
        GraspitCommander.approachToContact()
        GraspitCommander.setDynamics(True)
        self.assertTrue(GraspitCommander.getDynamics())

        GraspitCommander.autoGrasp()
        self.assertFalse(GraspitCommander.dynamicAutoGraspComplete())
        while not GraspitCommander.dynamicAutoGraspComplete():
            time.sleep(0.01)

        self.assertTrue(GraspitCommander.dynamicAutoGraspComplete())
        GraspitCommander.setDynamics(False)
        result = GraspitCommander.computeQuality()

        self.assertAlmostEqual(result.volume, 0.005218228, 4)
        self.assertAlmostEqual(result.epsilon, 0.018258844, 4)

    #TODO this test is somewhat trivial, as the hand is not in contact with
    # the object when findInitialContact is called.
    def testFindInitialContact(self):
        # Test equivalence of FindInitialContact and ApproachContact
        #  on the plannerMug world.
        GraspitCommander.loadWorld("plannerMug")
        GraspitCommander.findInitialContact()
        robot = GraspitCommander.getRobots().ids[0]
        pose_test = GraspitCommander.getRobot(robot).robot.pose
        GraspitCommander.clearWorld()

        GraspitCommander.loadWorld("plannerMug")
        GraspitCommander.approachToContact()
        robot = GraspitCommander.getRobots().ids[0]
        pose_target = GraspitCommander.getRobot(robot).robot.pose

        self.assertTrue(pose_test == pose_target)
