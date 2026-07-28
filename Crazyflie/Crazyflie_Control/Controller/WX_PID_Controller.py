# -*- coding: utf-8 -*-
"""
Created on Sun Dec 15 22:00:54 2024
@author: Wang Xu


二阶低通滤波器
pid控制器类
"""

import numpy as np
import matplotlib.pyplot as plt


class lpf2:
    """
    二阶低通滤波器
    """

    def __init__(self, sample_freq, cutoff_freq):
        """ Initialize and run the example with the specified link_uri """

        self.sample_freq = sample_freq
        self.cutoff_freq = cutoff_freq
        self.lpf2SetCutoffFreq()

    def lpf2SetCutoffFreq(self):
        fr = self.sample_freq / self.cutoff_freq
        ohm = np.tan(np.pi / fr)
        c = 1.0 + 2.0 * np.cos(np.pi / 4.0) * ohm + ohm * ohm
        self.b0 = ohm * ohm / c;
        self.b1 = 2.0 * self.b0;
        self.b2 = self.b0;
        self.a1 = 2.0 * (ohm * ohm - 1.0) / c;
        self.a2 = (1.0 - 2.0 * np.cos(np.pi / 4.0) * ohm + ohm * ohm) / c;
        self.delay_element_1 = 0.0;
        self.delay_element_2 = 0.0;

    def lpf2pApply(self, sample):
        self.delay_element_0 = sample - self.delay_element_1 * self.a1 - self.delay_element_2 * self.a2;
        if abs(sample) > 1e5: self.delay_element_0 = sample
        output = self.delay_element_0 * self.b0 + self.delay_element_1 * self.b1 + self.delay_element_2 * self.b2;
        self.delay_element_2 = self.delay_element_1;
        self.delay_element_1 = self.delay_element_0;
        return output

    def lpf2Reset(self, sample):
        dval = sample / (self.b0 + self.b1 + self.b2);
        self.delay_element_1 = dval;
        self.delay_element_2 = dval;
        return self.lpf2pApply(sample);


class PIDcontroller:

    def __init__(self, desired, kp, ki, kd, kff, dt, samplingRate, cutoffFreq, enableDFilter):
        """ Initialize and run the example with the specified link_uri """

        self.error = 0.0
        self.prevMeasured = 0.0
        self.integ = 0.0
        self.deriv = 0.0
        self.desired = desired
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kff = kff
        self.iLimit = 5000
        self.outputLimit = 0.0
        self.dt = dt
        self.enableDFilter = enableDFilter
        if (self.enableDFilter): self.lpf = lpf2(samplingRate, cutoffFreq)
        self.CONFIG_CONTROLLER_PID_FILTER_ALL = 1  # 定义为1时，不对微分项进行滤波，只对整体输出滤波

    def pidupdate(self, setpoint, measured, isYawAngle):
        output = 0.0
        self.desired = setpoint
        self.error = self.desired - measured;

        if (isYawAngle):
            if (self.error > 180.0):
                self.error -= 360.0
            elif (self.error < -180.0):
                self.error += 360.0

        self.outP = self.kp * self.error;
        output += self.outP;

        delta = -(measured - self.prevMeasured)

        if (isYawAngle):
            if (delta > 180.0):
                delta -= 360.0
            elif (delta < -180.0):
                delta += 360.0

        if self.CONFIG_CONTROLLER_PID_FILTER_ALL:
            self.deriv = delta / self.dt
        else:
            if (self.enableDFilter):
                self.deriv = self.lpf.lpf2pApply(delta / self.dt)
            else:
                self.deriv = delta / self.dt

        if (abs(self.deriv) > 1e5):
            self.deriv = 0

        self.outD = self.kd * self.deriv
        output += self.outD

        self.integ += self.error * self.dt

        if (abs(self.iLimit) > 0.01):
            self.integ = np.clip(self.integ, -self.iLimit, self.iLimit)

        self.outI = self.ki * self.integ
        output += self.outI

        self.outFF = self.kff * self.desired
        output += self.outFF

        if self.CONFIG_CONTROLLER_PID_FILTER_ALL:
            if (self.enableDFilter):
                output = self.lpf.lpf2pApply(output)
            else:
                output = output

        if (abs(self.outputLimit) > 0.01):  # 1220 修改bug，错误写成ilimit
            output = np.clip(output, -self.outputLimit, self.outputLimit)

        self.prevMeasured = measured

        return output


class AXis3f:

    def __init__(self):
        """  """
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class AXis6f:

    def __init__(self):
        """  """
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0


class Position_controller:

    def __init__(self, control_fre):
        """  """
        self.posFiltEnable = 1
        self.velFiltEnable = 1
        self.posFiltCutoff = 20.0
        self.velFiltCutoff = 20.0
        self.posZFiltEnable = 1
        self.velZFiltEnable = 1
        self.posZFiltCutoff = 20.0

        self.mode = 0  # mode为0是位置控制，接收位置控制指令；1为速度控制
        # 速度环PID解算时允许的最大角度 Maximum roll/pitch angle permited
        self.rLimit = 20.0
        self.pLimit = 20.0
        self.rpLimitOverhead = 1.10
        # 位置环PID的最大运行速度 Velocity maximums
        self.xVelMax = 1.0
        self.yVelMax = 1.0
        self.zVelMax = 1.0
        self.velMaxOverhead = 1.10

        self.thrustScale = 1000.0

        self.CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD = 1  # 如果定义z向滤波，滤波截止频率拉低

        if self.CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD:
            self.velZFiltCutoff = 5.7  # 原参数为0.7
            self.thrustBase = 38000
        else:
            self.velZFiltCutoff = 20.0
            self.thrustBase = 38000
        self.thrustMin = 20000

        self.PID_POS_X_KP = 2.0
        self.PID_POS_X_KI = 0.0
        self.PID_POS_X_KD = 0.0
        self.PID_POS_X_KFF = 0.0

        self.PID_POS_Y_KP = 2.0
        self.PID_POS_Y_KI = 0.0
        self.PID_POS_Y_KD = 0.0
        self.PID_POS_Y_KFF = 0.0

        self.PID_POS_Z_KP = 5.0
        self.PID_POS_Z_KI = 0.0
        self.PID_POS_Z_KD = 0.0
        self.PID_POS_Z_KFF = 0.0

        self.PID_VEL_X_KP = 25.0
        self.PID_VEL_X_KI = 1.0
        self.PID_VEL_X_KD = 0.0
        self.PID_VEL_X_KFF = 0.0

        self.PID_VEL_Y_KP = 25.0
        self.PID_VEL_Y_KI = 1.0
        self.PID_VEL_Y_KD = 0.0
        self.PID_VEL_Y_KFF = 0.0

        self.PID_VEL_Z_KP = 25.0
        self.PID_VEL_Z_KI = 1.0
        self.PID_VEL_Z_KD = 1.7
        self.PID_VEL_Z_KFF = 0.0

        self.Position_Rate = control_fre + 0.0
        self.dt = 1 / self.Position_Rate

        self.pidVX = PIDcontroller(0.0, self.PID_VEL_X_KP, self.PID_VEL_X_KI, self.PID_VEL_X_KD, self.PID_VEL_X_KFF, \
                                   self.dt, self.Position_Rate, self.velFiltCutoff, self.velFiltEnable)
        self.pidVY = PIDcontroller(0.0, self.PID_VEL_Y_KP, self.PID_VEL_Y_KI, self.PID_VEL_Y_KD, self.PID_VEL_Y_KFF, \
                                   self.dt, self.Position_Rate, self.velFiltCutoff, self.velFiltEnable)
        self.pidVZ = PIDcontroller(0.0, self.PID_VEL_Z_KP, self.PID_VEL_Z_KI, self.PID_VEL_Z_KD, self.PID_VEL_Z_KFF, \
                                   self.dt, self.Position_Rate, self.velZFiltCutoff, self.velZFiltEnable)

        self.pidX = PIDcontroller(0.0, self.PID_POS_X_KP, self.PID_POS_X_KI, self.PID_POS_X_KD, self.PID_POS_X_KFF, \
                                  self.dt, self.Position_Rate, self.posFiltCutoff, self.posFiltEnable)
        self.pidY = PIDcontroller(0.0, self.PID_POS_Y_KP, self.PID_POS_Y_KI, self.PID_POS_Y_KD, self.PID_POS_Y_KFF, \
                                  self.dt, self.Position_Rate, self.posFiltCutoff, self.posFiltEnable)
        self.pidZ = PIDcontroller(0.0, self.PID_POS_Z_KP, self.PID_POS_Z_KI, self.PID_POS_Z_KD, self.PID_POS_Z_KFF, \
                                  self.dt, self.Position_Rate, self.posZFiltCutoff, self.posZFiltEnable)

        self.setpoint_velocity = AXis3f()

    def positionController(self, state, setpoint):
        self.pidX.outputLimit = self.xVelMax * self.velMaxOverhead
        self.pidY.outputLimit = self.yVelMax * self.velMaxOverhead

        self.pidZ.outputLimit = max(self.zVelMax, 0.5) * self.velMaxOverhead;

        cosyaw = np.cos(state.yaw * np.pi / 180.0)
        sinyaw = np.sin(state.yaw * np.pi / 180.0)

        setp_body_x = setpoint.x * cosyaw + setpoint.y * sinyaw
        setp_body_y = -setpoint.x * sinyaw + setpoint.y * cosyaw

        state_body_x = state.x * cosyaw + state.y * sinyaw
        state_body_y = -state.x * sinyaw + state.y * cosyaw

        globalvx = setpoint.vx
        globalvy = setpoint.vy

        self.setpoint_velocity.x = setpoint.vx;
        self.setpoint_velocity.y = setpoint.vy;
        self.setpoint_velocity.z = setpoint.vz;

        if (self.mode == 0):
            self.setpoint_velocity.x = self.pidX.pidupdate(setp_body_x, state_body_x, False)
            self.setpoint_velocity.y = self.pidY.pidupdate(setp_body_y, state_body_y, False)
            self.setpoint_velocity.z = self.pidZ.pidupdate(setpoint.z, state.z, False)
        elif (self.mode == 1):
            self.setpoint_velocity.x = globalvx * cosyaw + globalvy * sinyaw;
            self.setpoint_velocity.y = globalvy * cosyaw - globalvx * sinyaw;

        # print(self.setpoint_velocity.x, self.setpoint_velocity.y, self.setpoint_velocity.z)

        thrust, pitch, roll = self.velocityController(self.setpoint_velocity, state)
        return thrust, pitch, roll

    def velocityController(self, setpoint_velocity, state):
        self.pidVX.outputLimit = self.pLimit * self.rpLimitOverhead;
        self.pidVY.outputLimit = self.rLimit * self.rpLimitOverhead;
        # Set the output limit to the maximum thrust range
        self.pidVZ.outputLimit = (65535 / 2 / self.thrustScale);
        # this.pidVZ.pid.outputLimit = (this.thrustBase - this.thrustMin) / thrustScale;

        cosyaw = np.cos(state.yaw * np.pi / 180.0);
        sinyaw = np.sin(state.yaw * np.pi / 180.0);
        state_body_vx = state.vx * cosyaw + state.vy * sinyaw;
        state_body_vy = -state.vx * sinyaw + state.vy * cosyaw;

        # Roll and Pitch
        pitch = -self.pidVX.pidupdate(setpoint_velocity.x, state_body_vx, False)
        roll = -self.pidVY.pidupdate(setpoint_velocity.y, state_body_vy, False);

        roll = np.clip(roll, -self.rLimit, self.rLimit)
        pitch = np.clip(pitch, -self.pLimit, self.pLimit)

        thrustRaw = self.pidVZ.pidupdate(setpoint_velocity.z, state.vz, False)
        # Scale the thrust and add feed forward term
        thrust = thrustRaw * self.thrustScale + self.thrustBase
        # Check for minimum thrust
        if (thrust < self.thrustMin): thrust = self.thrustMin;

        thrust = np.clip(thrust, 0, 62000)

        return thrust, pitch, roll


if __name__ == '__main__':
    sample_freq = 500.0
    cufoff_freq = 10.0
    testLpf2 = lpf2(sample_freq, cufoff_freq)
    testLpf2v = lpf2(sample_freq, 4)
    test_time = 10.0
    test_len = int(test_time * sample_freq)
    state_infromation = np.zeros([test_len, 4])
    last_obs = 0
    for i in range(test_len):
        obs = np.sin(i / sample_freq) + 0.5 * np.random.rand()
        lp2f_state = testLpf2.lpf2pApply(obs)
        delta = (obs - last_obs) * sample_freq
        div_state = testLpf2v.lpf2pApply(delta)
        state_infromation[i] = np.array([obs, lp2f_state, delta, div_state])
        last_obs = obs + 0.0
    plt.subplot(2, 1, 1)
    plt.plot(np.arange(len(state_infromation[:, 0])), state_infromation[:, 0], label='obs')
    plt.plot(np.arange(len(state_infromation[:, 0])), state_infromation[:, 1], label='fl')

    plt.subplot(2, 1, 2)
    # plt.plot(np.arange(len(state_infromation[:,0])),state_infromation[:,2],label='obs_v')
    plt.plot(np.arange(len(state_infromation[:, 0])), state_infromation[:, 3], label='fl_v')

    plt.ylabel('Lp2f')
    plt.legend()
    plt.show()
