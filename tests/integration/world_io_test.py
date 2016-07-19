import unittest

from graspit_commander.graspit_exceptions import (
    LoadWorldException
)

from graspit_commander.graspit_commander import GraspitCommander


class WorldIOTest(unittest.TestCase):

    ROS_SERVICE_TIMEOUT = 1

    """Make sure GraspIt! has a clean world and we can connect to it"""
    def setUp(self):
        GraspitCommander.clearWorld()

    def testLoadWorld(self):
        GraspitCommander.loadWorld("plannerMug")

    def testLoadInvalidWorld(self):
        with self.assertRaises(LoadWorldException):
            GraspitCommander.loadWorld("")

    def testImportObstacle(self):
        GraspitCommander.importObstacle("floor")

    def testImportInvalidObstacle(self):
        with self.assertRaises(Exception):
            GraspitCommander.importObstacle("")

    def testImportGraspableBody(self):
        GraspitCommander.importGraspableBody("ashtray")

    def testImportInvalidGraspableBody(self):
        with self.assertRaises(Exception):
            GraspitCommander.importGraspableBody("")
