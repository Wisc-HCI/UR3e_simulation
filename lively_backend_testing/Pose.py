class Pose:
    def __init__(self, position=(0, 0, 0), orientation=(0, 0, 0, 1)):
        self.position = position  # A tuple (x, y, z)
        self.orientation = orientation  # A tuple representing quaternion (x, y, z, w)