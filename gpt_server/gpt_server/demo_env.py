import numpy as np

from robopal.envs import RobotEnv
from robopal.robots.diana_med import DianaGraspMultiObjs

def primitive(func, checker=None):
    """ primitive flag, no practical effect. """

    def primitive_wrapper(*args, **kwargs):
        return func(*args, **kwargs)

    return primitive_wrapper


class GraspingEnv(RobotEnv):
    def __init__(self,
                 robot=DianaGraspMultiObjs,
                 render_mode="human",
                 control_freq=20,
                 is_interpolate=False,
                 controller="CARTIK",
                 ):
        super().__init__(
            robot=robot,
            render_mode=render_mode,
            control_freq=control_freq,
            is_interpolate=is_interpolate,
            controller=controller,
        )

        self.init_pos, self.init_rot = self.controller.forward_kinematics(self.robot.get_arm_qpos())
        self.action = self.init_pos

    @primitive
    def reset_robot(self):
        self.move(self.init_pos, self.init_rot)

    @primitive
    def get_obj_pose(self, obj_name):
        pos = self.mj_data.body(obj_name).xpos.copy()
        pos[2] -= 0.31
        quat = self.mj_data.body(obj_name).xquat.copy()
        return pos, quat

    @primitive
    def move(self, pos, quat):
        def checkArriveState(state):
            current_pos, current_quat = self.get_current_pose()
            error = np.sum(np.abs(state[:3] - current_pos)) + np.sum(np.abs(state[3:] - current_quat))
            if error <= 0.02:
                return True
            return False

        while True:
            self.action = np.concatenate((pos, quat), axis=0)
            self.step(self.action)
            if checkArriveState(self.action):
                break

    @primitive
    def grab(self, obj_name):
        self.gripper_ctrl("open")
        obj_pos, obj_quat = self.get_obj_pose(obj_name)
        self.move(np.add(obj_pos, np.array([0, 0, 0.1])), obj_quat)
        self.move(obj_pos, obj_quat)
        self.gripper_ctrl("close")
        end_pos, end_quat = self.get_current_pose()
        self.move(np.add(end_pos, np.array([0, 0, 0.1])), end_quat)

    @primitive
    def gripper_ctrl(self, cmd: str):
        if cmd == "open":
            self.mj_data.actuator("0_gripper_l_finger_joint").ctrl = 20
        elif cmd == "close":
            self.mj_data.actuator("0_gripper_l_finger_joint").ctrl = -20
        step = 0
        while True:
            step += 1
            self.step(self.action)
            if step > 80:
                break

    @primitive
    def get_current_pose(self):
        return self.controller.forward_kinematics(self.robot.get_arm_qpos())


def make_env():
    env = GraspingEnv(
        render_mode="human",
        control_freq=100,
    )
    return env

if __name__ == "__main__":
    env = make_env()
    env.reset()
    env.move(np.array([0.6, 0.3, 0.2]), np.array([1, 0, 0, 0]))
    env.close()
