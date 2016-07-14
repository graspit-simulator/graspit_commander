import unittest

from graspit_commander.graspit_commander import GraspitCommander


class PlannerTest(unittest.TestCase):

    ROS_SERVICE_TIMEOUT = 1

    """Make sure GraspIt! has a clean world and we can connect to it"""
    def setUp(self):
        GraspitCommander.clearWorld()

    def testSimAnnPlannerOnPlannerMugWorld(self):
        GraspitCommander.loadWorld("plannerMug")
        r = GraspitCommander.planGrasps(max_steps=60000)
        self.assertGreater(len(r.grasps), 0)
