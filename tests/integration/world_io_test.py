import unittest

from graspit_commander.graspit_exceptions import (
    GraspitTimeoutException,
    LoadWorldException
)

from graspit_commander.graspit_commander import GraspitCommander


class WorldIOTest(unittest.TestCase):

    ROS_SERVICE_TIMEOUT = 1

    """Make sure GraspIt! has a clean world and we can connect to it"""
    def setUp(self):
        try:
            GraspitCommander.clearWorld()
        except GraspitTimeoutException, e:
            print "Connection to Graspit! is failing. Make sure you are running GraspIt! before running these tests."

    def testLoadWorld(self):
        GraspitCommander.loadWorld("plannerMug")

    def testLoadInvalidWorld(self):
        with self.assertRaises(LoadWorldException):
            GraspitCommander.loadWorld("")
