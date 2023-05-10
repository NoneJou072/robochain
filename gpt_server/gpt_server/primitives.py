"""
    PrimitiveSet contains low level primitives called by assistant.
    It has a one-to-one correspondence with primitives.txt prompts' file.
"""
class PrimitiveSet():
    def __init__(self) -> None:
        pass

    def openGripper(self, angle):
        """
            open the gripper.
        """
        print("open the gripper to {}".format(angle))
        
    def closeGripper(self, angle):
        """
            close the gripper
        """
        print("close gripper.")

    def updateTargetPose(self, target):
        """
            update the target pose
        """
        print("update target pose.")
