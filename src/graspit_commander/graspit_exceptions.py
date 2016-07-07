class InvalidIDException(Exception):
    def __init__(self):
        Exception.__init__(self, "Invalid ID")


class InvalidPoseException(Exception):
    def __init__(self):
        Exception.__init__(self, "Invalid Pose")


class CollisionException(Exception):
    def __init__(self):
        Exception.__init__(self, "Collision")


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
