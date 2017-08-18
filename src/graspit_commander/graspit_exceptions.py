class GraspitTimeoutException(Exception):
    def __init__(self, service=None):
        if service is not None:
            Exception.__init__(self, "Timeout! Could not connect to service '{0}'. Are you sure GraspIt! is running?".format(service))
        else:
            Exception.__init__(self, "Timeout! Are you sure GraspIt! is running?")
        self.service = service


class InvalidIDException(Exception):
    def __init__(self):
        Exception.__init__(self, "Invalid ID")


class InvalidPoseException(Exception):
    def __init__(self):
        Exception.__init__(self, "Invalid Pose")


class CollisionException(Exception):
    def __init__(self):
        Exception.__init__(self, "Collision")


class ImportException(Exception):
    def __init__(self, details=""):
        if details != "": details = ": "+details
        Exception.__init__(self, "Import failed%s" % details)


class ImportRobotException(ImportException):
    def __init__(self, name=""):
        ImportException.__init__(self, details="for robot %s" % name)


class ImportBodyException(ImportException):
    def __init__(self, body_type= "body", details=""):
        ImportException.__init__(self, "for %s %s" % (body_type, details))


class ImportObstacleException(ImportBodyException):
    def __init__(self, name=""):
        ImportBodyException.__init__(self, body_type="obstacle",
                                     details=name)

class ImportGraspableBodyException(ImportBodyException):
    def __init__(self, name=""):
        ImportBodyException.__init__(self, body_type="graspable body",
                                     details=name)

class InvalidRobotIDException(InvalidIDException):
    def __init__(self, id=None):
        if id is not None:
            Exception.__init__(self, "Invalid robot ID specified (id={0})".format(id))
        else:
            Exception.__init__(self, "Invalid robot ID specified")

        self.id = id


class InvalidRobotPoseException(InvalidPoseException):
    def __init__(self):
        Exception.__init__(self, "Invalid robot pose specified")


class RobotCollisionException(InvalidRobotPoseException, CollisionException):
    def __init__(self):
        Exception.__init__(self, "Invalid pose. Will put robot in collision")


class InvalidRobotDOFOutOfRangeException(Exception):
    def __init__(self):
        Exception.__init__(self, "DOF values out of acceptable range")


class InvalidRobotDOFCountMismatchException(Exception):
    def __init__(self):
        Exception.__init__(self, "Number of DOFs does not match selected robot")


class InvalidGraspableBodyIDException(InvalidIDException):
    def __init__(self, id=None):
        if id is not None:
            Exception.__init__(self, "Invalid graspable body ID specified (id={0})".format(id))
        else:
            Exception.__init__(self, "Invalid graspable body ID specified")

        self.id = id


class InvalidGraspableBodyPoseException(InvalidPoseException):
    def __init__(self):
        Exception.__init__(self, "Invalid graspable body pose specified")


class GraspableBodyCollisionException(InvalidGraspableBodyPoseException, CollisionException):
    def __init__(self):
        Exception.__init__(self, "Invalid pose. Will put graspable body in collision")


class InvalidBodyIDException(InvalidIDException):
    def __init__(self, id=None):
        if id is not None:
            Exception.__init__(self, "Invalid body ID specified (id={0})".format(id))
        else:
            Exception.__init__(self, "Invalid body ID specified")

        self.id = id


class InvalidBodyPoseException(InvalidPoseException):
    def __init__(self):
        Exception.__init__(self, "Invalid body pose specified")


class BodyCollisionException(InvalidGraspableBodyPoseException, CollisionException):
    def __init__(self):
        Exception.__init__(self, "Invalid pose. Will put body in collision")


class ClearWorldException(Exception):
    def __init__(self):
        Exception.__init__(self, "Could not clear world")


class LoadWorldException(Exception):
    def __init__(self):
        Exception.__init__(self, "Could not load world")


class SaveImageException(Exception):
    def __init__(self):
        Exception.__init__(self, "Could not save image.")


class SaveWorldException(Exception):
    def __init__(self):
        Exception.__init__(self, "Could not save world.")


class InvalidDynamicsModeException(Exception):
    def __init__(self):
        Exception.__init__(self, "Current Dynamics Mode is not compatible with this service. Toggle Dynamics and try again.")


class InvalidEnergyTypeException(Exception):
    def __init__(self, enery_type):
        Exception.__init__(self, "Invalid Energy Type specified (id={0})".format(enery_type))
