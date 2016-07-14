import unittest

from graspit_commander.graspit_commander import GraspitCommander


class GraspingTest(unittest.TestCase):

    ROS_SERVICE_TIMEOUT = 1

    """Make sure GraspIt! has a clean world and we can connect to it"""
    def setUp(self):
        GraspitCommander.clearWorld()

    def testFullStaticGraspExecution(self):
        GraspitCommander.loadWorld("plannerMug")
        GraspitCommander.approachToContact()
        GraspitCommander.autoGrasp()
        result = GraspitCommander.computeQuality()

        self.assertAlmostEqual(result.volume, 0.004336969, 4)
        self.assertAlmostEqual(result.epsilon, 0.046997464, 4)

