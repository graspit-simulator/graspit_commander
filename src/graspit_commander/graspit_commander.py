import rospy

from graspit_interface.srv import GetRobot

class GraspitCommander(object): 
    """
    Python interface for interacting with GraspIt
    """
   
    @staticmethod
    def getRobot(i):
        rospy.wait_for_service('getRobot')
        
        serviceProxy = rospy.ServiceProxy('getRobot', GetRobot)
        robot = serviceProxy(i)

        return robot
