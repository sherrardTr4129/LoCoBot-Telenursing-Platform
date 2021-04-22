class FrontEndState:
    """
    This class holds the various state values of the 
    robot as represented in the front-end application
    """
    def __init__(self):
        self.fwdRev = 0
        self.spin = 0
        self.armOffset_x = 0
        self.armOffset_y = 0
        self.armOffset_z = 0
        self.panOffset = 0
        self.tiltOffset = 0
        self.gripperState = 0

    def __eq__(self, other):
        return (self.fwdRev == other.fwdRev and self.spin == other.spin and \
                self.armOffset_x == other.armOffset_x and self.armOffset_y == other.armOffset_y \
                and self.armOffset_z == other.armOffset_z and self.panOffset == other.panOffset and \
                self.tiltOffset == other.tiltOffset and self.gripperState == other.gripperState)

    def __iter__(self):
        for field, val in self.__dict__.items():
            yield field, val
    
    def set_fwd_rev(self, val):
        self.fwdRev = val

    def set_spin(self, val):
        self.spin = val

    def set_arm_offset_x(self, val):
        self.armOffset_x = val

    def set_arm_offset_y(self, val):
        self.armOffset_y = val

    def set_arm_offset_z(self, val):
        self.armOffset_z = val

    def set_pan_offset(self, val):
        self.panOffset = val

    def set_tilt_offset(self, val):
        self.tiltOffset = val

    def set_gripper_state(self, val):
        self.gripperState = val
