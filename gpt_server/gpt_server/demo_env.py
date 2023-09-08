import numpy as np
# from roboimi.envs.pos_ctrl_env import PosCtrlEnv
from robopal.envs.task_ctrl_env import PosCtrlEnv
# import roboimi.utils.KDL_utils.transform as T


def primitive(func, checker=None):
    """ primitive flag, no practical effect. """

    def primitive_wrapper(*args, **kwargs):
        return func(*args, **kwargs)

    return primitive_wrapper


class GraspingEnv(PosCtrlEnv):
    def __init__(self,
                 robot=None,
                 is_render=True,
                 renderer="viewer",
                 control_freq=20,
                 is_interpolate=True,
                 is_pd=True,
                 ):
        super().__init__(
            robot=robot,
            is_render=is_render,
            renderer=renderer,
            control_freq=control_freq,
            is_interpolate=is_interpolate,
            is_pd=is_pd
        )

        self.action = None

        can_list = ['red_block', 'blue_block', 'green_block']
        self.can_pos, self.can_quat = self.getObjPose(can_list)

    def reset(self):
        super().reset()
        init_pos, init_rot = self.kdl_solver.fk(self.robot.single_arm.arm_qpos, rot_format='quat')
        self.action = init_pos.copy()
        self.move(init_pos, self.can_quat['blue_block'])

    def getObjPose(self, name_list):
        pos = {}
        quat = {}
        for name in name_list:
            pos[name] = self.mj_data.body(name).xpos.copy()
            pos[name][2] -= 0.20
            quat[name] = self.mj_data.body(name).xquat.copy()
        return pos, quat

    @primitive
    def move(self, pos, quat):
        def checkArriveState(state):
            current_pos, current_quat = self.getCurrentPose()
            error = np.sum(np.abs(state[:3] - current_pos)) + np.sum(np.abs(state[3:] - current_quat))
            if error <= 0.01:
                return True
            return False

        while True:
            self.action = np.concatenate((pos, quat), axis=0)
            self.step(self.action)
            if self.is_render:
                self.render()
            if checkArriveState(self.action):
                break

    @primitive
    def gripper_ctrl(self, cmd: str):
        if cmd == "open":
            self.mj_data.actuator("0_gripper_l_finger_joint").ctrl = 20
        elif cmd == "close":
            self.mj_data.actuator("0_gripper_l_finger_joint").ctrl = -20
        step = 0
        while True:
            step += 1
            cur_pos, cur_quat = self.getCurrentPose()
            action = np.concatenate((cur_pos, cur_quat), axis=0)
            self.step(action)
            if self.is_render:
                self.render()
            if step > 60:
                break

    @primitive
    def getCurrentPose(self):
        current_pos, current_quat = self.kdl_solver.fk(self.robot.single_arm.arm_qpos, rot_format='quat')
        return current_pos, current_quat

def make_env():
    # from roboimi.assets.robots.diana_grasp import DianaMed
    from robopal.assets.robots.diana_med import DianaGrasp
    env = GraspingEnv(
        robot=DianaGrasp(),
        renderer="viewer",
        is_render=True,
        control_freq=200,
        is_interpolate=False,
        is_pd=True
    )
    return env
