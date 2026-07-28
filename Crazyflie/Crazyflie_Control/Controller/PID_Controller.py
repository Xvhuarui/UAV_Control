# -*- coding: utf-8 -*-
import numpy as np


class AXis6f:
    """数据容器：用于存放主程序传进来的实际状态和目标状态"""

    def __init__(self):
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.vx, self.vy, self.vz = 0.0, 0.0, 0.0
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0


class PositionController:
    def __init__(self, control_freq):
        # ---------- 基础时间参数 ----------
        self.dt = 1.0 / control_freq

        # 必须保留的空壳变量，供主程序画图读取
        self.setpoint_velocity = AXis6f()

        # ---------- 物理与硬件参数 (实机特有) ----------
        # 悬停基准推力 (非常关键！实机依靠这个抵抗重力，通常满电在 38000 左右)
        self.hover_thrust = 38000
        # 用于角度映射的归一化重力常数 (对应你仿真里的 m*g)
        self.mass_g = 9.8

        # ---------- PID 增益参数 (需根据实机微调) ----------
        # X/Y 轴增益 (输出为虚拟力)
        self.p_x, self.i_x, self.d_x = 2.0, 0.0, 1.5
        self.p_y, self.i_y, self.d_y = 2.0, 0.0, 1.5

        # Z 轴增益 (输出直接是 PWM 增量，量级通常数以千计)
        self.p_z, self.i_z, self.d_z = 25000.0, 1000.0, 1700.0

        # ---------- 控制参数初始化 ----------
        self.x_err_pre, self.y_err_pre, self.z_err_pre = 0.0, 0.0, 0.0
        self.x_err_sum, self.y_err_sum, self.z_err_sum = 0.0, 0.0, 0.0

    def main_controller(self, state, setpoint):
        """
        核心控制函数
        输入：当前状态 state, 期望位置 setpoint
        输出：推力(0~65535), 俯仰角(度), 滚转角(度)
        """
        # ================= 1. 计算全局误差 =================
        x_err_global = setpoint.x - state.x
        y_err_global = setpoint.y - state.y
        z_err = setpoint.z - state.z

        # ================= 2. 偏航角(Yaw)坐标系旋转变换 =================
        # 这是实机必须加的一步：把“东南西北”的误差，旋转成无人机“前后左右”的误差
        yaw_rad = np.radians(state.yaw)
        cosyaw = np.cos(yaw_rad)
        sinyaw = np.sin(yaw_rad)

        x_err = x_err_global * cosyaw + y_err_global * sinyaw
        y_err = -x_err_global * sinyaw + y_err_global * cosyaw

        # ================= 3. 计算误差微分 =================
        # 相当于你仿真里的 v_x_err
        v_x_err = (x_err - self.x_err_pre) / self.dt
        v_y_err = (y_err - self.y_err_pre) / self.dt
        v_z_err = (z_err - self.z_err_pre) / self.dt

        # 将误差微分赋给 setpoint_velocity 供外部主程序画图
        self.setpoint_velocity.x = v_x_err
        self.setpoint_velocity.y = v_y_err
        self.setpoint_velocity.z = v_z_err

        # ================= 4. PID 核心算式 (完美复刻你的仿真逻辑) =================
        T_x = self.p_x * x_err + self.i_x * self.x_err_sum + self.d_x * v_x_err
        T_y = self.p_y * y_err + self.i_y * self.y_err_sum + self.d_y * v_y_err
        T_z_pid = self.p_z * z_err + self.i_z * self.z_err_sum + self.d_z * v_z_err

        # 更新误差历史 (加入积分限幅，防止积分饱和炸机)
        self.x_err_sum = np.clip(self.x_err_sum + x_err * self.dt, -2.0, 2.0)
        self.y_err_sum = np.clip(self.y_err_sum + y_err * self.dt, -2.0, 2.0)
        self.z_err_sum = np.clip(self.z_err_sum + z_err * self.dt, -5.0, 5.0)

        self.x_err_pre = x_err
        self.y_err_pre = y_err
        self.z_err_pre = z_err

        # ================= 5. 指令映射与安全限幅 =================
        # 1. Z轴高度映射：基准悬停推力 + PID 补偿
        thrust = int(self.hover_thrust + T_z_pid)
        thrust = np.clip(thrust, 10000, 60000)  # 严格限制在底层电调允许的 PWM 范围内

        # 2. X/Y轴姿态映射：将虚拟力转化为目标角度 (复刻仿真公式)
        # 为防止除以 0 或除以负数导致角度翻转，实机中常把 T_z 近似为固定的重力常数
        phi_tar_rad = -np.arctan(T_y / self.mass_g)
        theta_tar_rad = np.arctan(T_x / self.mass_g)

        # 3. 弧度转角度，并限制最大倾角防翻车
        roll = np.clip(np.degrees(phi_tar_rad), -20.0, 20.0)
        pitch = np.clip(np.degrees(theta_tar_rad), -20.0, 20.0)

        return thrust, pitch, roll