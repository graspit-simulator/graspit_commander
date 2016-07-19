import unittest

from graspit_commander.graspit_exceptions import *


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
        with self.assertRaises(ImportException):
            GraspitCommander.importObstacle("")

    def testImportGraspableBody(self):
        GraspitCommander.importGraspableBody("ashtray")

    def testImportInvalidGraspableBody(self):
        with self.assertRaises(ImportException):
            GraspitCommander.importGraspableBody("")

    def testImportRobot(self):
        GraspitCommander.importRobot("Barrett")

    def testImportInvalidRobot(self):
        with self.assertRaises(ImportException):
            GraspitCommander.importRobot("")

    def testSaveImage(self):
        GraspitCommander.loadWorld("plannerMug")
        GraspitCommander.saveImage("test_img")

    def testSaveWorld(self):
        GraspitCommander.importRobot("Barrett")
        GraspitCommander.importGraspableBody("ashtray")
        GraspitCommander.saveWorld("test_world")

    def testToggleAllCollisions(self):
        GraspitCommander.toggleAllCollisions(True)
