# -*- coding: utf-8 -*-
"""
Created on Mon Jul 15 10:59:32 2026
@author: XHR
"""
# =============================== 库导入 ===============================
# ---------- 常规库导入 ----------
import logging
import time
from threading import Timer, Thread
import numpy as np
import matplotlib.pyplot as plt
import os
# ---------- 动捕库导入 ----------
from nokov.nokovsdk import *

nokovIp = '10.1.1.198'
nokov_client = PySDKClient()
ret = nokov_client.Initialize(bytes(nokovIp, encoding="utf8"))
# ---------- 通讯库导入 ----------
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

uriIp = 'radio://0/80/2M/E7E7E7E7E7'
uri = uri_helper.uri_from_env(default = uriIp)
logging.basicConfig(level = logging.ERROR)
# ---------- 个人库导入 ----------
from Controller.WX_PID_Controller import Position_controller, AXis6f

from Utilities.utils_plot import UAVPlotter
from Utilities.utils_data import DataLoggerIdeal

plotter = UAVPlotter()
logger_dir = r'E:\test\Crazyflie\Crazyflie_Control\Simulation_Data\GroundEffect_Nokov\Figure8\RecordData'
logger = DataLoggerIdeal(output_dir=logger_dir)
# =============================== 参数设置 ===============================
global_cf = None
debug_counter = 0
last_extpos_time = 0

ControlTime_in_ms = 10
Control_freq = 1000.0 / ControlTime_in_ms
dt = 1 / Control_freq
mass = 0.039

# ---------- 八字形轨迹参数 ----------
low_z    = 0.06           # 起始爬升高度
high_z   = 0.23           # 结束爬升高度
stable_z = 0.10           # 最终回旋悬停高度

fig8_A = 0.8              # 八字形 X 轴基础振幅 (米)
fig8_B = 0.4              # 八字形 Y 轴基础振幅 (米)
omega  = 0.8              # 飞行角速度
rotation_angle_deg = 315.0 # 顺时针旋转角度 (度)，正数为顺时针

# 转换为弧度 (顺时针在标准坐标系中为负角)
rot_rad = -rotation_angle_deg * np.pi / 180.0
cos_rot = np.cos(rot_rad)
sin_rot = np.sin(rot_rad)

# ---------- 步长设置 ----------
hover_step_len  = 800
fig8_step_len   = 6000      # 画八字并上升的持续时间
return_step_len = 2000    # 缩回原点的持续时间
end_step_len    = 200
step = hover_step_len + fig8_step_len + return_step_len + end_step_len

flight_time = (ControlTime_in_ms * step) / 1000.0
control_whole_time = flight_time + 1

R = 0.023
D = 2 * R

data_len = int(control_whole_time * Control_freq)
state_information      = np.zeros([data_len, 12])
target_pos_information = np.zeros([data_len, 3])
target_alt_information = np.zeros([data_len, 3])
target_vel_information = np.zeros([data_len, 3])
control_information    = np.zeros([data_len, 5])
acc_information        = np.zeros([data_len, 3])
battery_information    = np.zeros([data_len, 1])

# =============================== 动捕函数 ===============================
def nokov_to_cf_callback(pFrameOfMocapData, pUserData):
# 动捕数据接收回调：只注入位置(X, Y, Z)
    global global_cf, debug_counter, last_extpos_time
    if pFrameOfMocapData is None or global_cf is None:
        return

    # 频率节流 (限制在约 50Hz，防止无线电缓冲区溢出)
    current_time = time.time()
    if current_time - last_extpos_time < 0.02:
        return
    last_extpos_time = current_time

    frameData = pFrameOfMocapData.contents

    if frameData.nRigidBodies > 0:
        body = frameData.RigidBodies[0]

        # 单位换算
        nokov_x = body.x / 1000.0
        nokov_y = body.y / 1000.0
        nokov_z = body.z / 1000.0

        # 变量映射
        cf_x = nokov_x
        cf_y = nokov_y
        cf_z = nokov_z

        debug_counter += 1
        if debug_counter % 10 == 0:
            print(f"[Nokov Debug] 纯位置数据已注入CF -> X:{cf_x:.4f}m, Y:{cf_y:.4f}m, Z:{cf_z:.4f}m")

        # 位置信息发送
        try:
            global_cf.extpos.send_extpos(cf_x, cf_y, cf_z)
        except Exception as e:
            if debug_counter % 50 == 0:
                print(f"[CF Error] 数据注入飞控失败: {e}")

# =============================== Crazyflie 主控类 ===============================
class CrazyflieTest:
# 负责通信、参数接管、日志记录以及调用外部 WX-PID

    def __init__(self, link_uri):
        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)
        self._cf.open_link(link_uri)

        self.is_connected = True
        self.i = 0
        self.is_shutting_down = False

        self.current_rej_count = -1
        self.has_rej_log = False

        # ================= WX-PID 控制相关类的初始化 =================
        self.control = Position_controller(Control_freq)
        self.state = AXis6f()
        self.set_point = AXis6f()
        # ==========================================================

        # 将实例暴露给全局，允许 Nokov 回调函数往里面发数据
        global global_cf
        global_cf = self._cf

    def _connected(self, link_uri):
        print('Successfully connected to %s' % link_uri)
        Thread(target=self._initialization_sequence).start()

    def _initialization_sequence(self):
        print("正在配置并重置内部状态估计器，请保持无人机静止...")

        # ================== 开启 EKF 上帝模式 (彻底抛弃气压计) ==================
        print(">>> 正在执行底层接管：屏蔽气压计 + 强行接纳动捕...")

        # 0. 强制激活卡尔曼滤波器
        try:
            self._cf.param.set_value('stabilizer.estimator', '2')
            print("    [成功] 强制激活 Kalman 滤波器 (stabilizer.estimator=2)")
        except KeyError:
            try:
                self._cf.param.set_value('estimator.estimator', '2')
                print("    [成功] 强制激活 Kalman 滤波器 (estimator.estimator=2)")
            except KeyError:
                print("    [警告] 无法通过代码激活 Kalman 滤波器")

        # 彻底禁用离群值拒绝
        outlier_params = {'kalman.ext_pos_mah': '1000.0', 'locSrv.extPosMaxDist': '1000.0',
                          'kalman.posOutlierReject': '0'}
        for p, v in outlier_params.items():
            try:
                self._cf.param.set_value(p, v)
                print(f"    [成功] 禁用离群值拦截 -> {p}")
            except KeyError:
                pass

        # 彻底屏蔽气压计
        baro_params = {'kalman.pZStdDev': '100.0', 'locSrv.baroStdDev': '100.0', 'baro.stdDev': '100.0'}
        for p, v in baro_params.items():
            try:
                self._cf.param.set_value(p, v)
                print(f"    [成功] 已彻底屏蔽气压计数据 -> {p}")
            except KeyError:
                pass

        # 提升外部动捕的绝对信任度
        std_params = ['locSrv.extPosStdDev', 'kalman.ext_pos_std', 'kalman.extPosStdDev']
        for p in std_params:
            try:
                self._cf.param.set_value(p, '0.01')
                print(f"    [成功] 动捕信任度已拉满 -> {p}")
                break
            except KeyError:
                pass

        time.sleep(0.1)

        # 连续深度复位 EKF
        print(">>> 正在进行深度复位...")
        try:
            for _ in range(2):
                self._cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.2)
                self._cf.param.set_value('kalman.resetEstimation', '0')
                time.sleep(0.5)
            print(">>> 复位完成！")
        except KeyError:
            print(">>> 提示: 未找到 kalman 显式复位参数，已跳过。")

        time.sleep(1.0)
        print("初始化完成，准备起飞！")
        print("=" * 100)
        # ---------- 卡车1: 状态估计日志 ----------
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=ControlTime_in_ms)
        self._lg_stab.add_variable('stateEstimate.x', 'FP16')
        self._lg_stab.add_variable('stateEstimate.y', 'FP16')
        self._lg_stab.add_variable('stateEstimate.z', 'FP16')
        self._lg_stab.add_variable('stateEstimate.vx', 'FP16')
        self._lg_stab.add_variable('stateEstimate.vy', 'FP16')
        self._lg_stab.add_variable('stateEstimate.vz', 'FP16')
        self._lg_stab.add_variable('stabilizer.roll', 'FP16')
        self._lg_stab.add_variable('stabilizer.pitch', 'FP16')
        self._lg_stab.add_variable('stabilizer.yaw', 'FP16')
        self._lg_stab.add_variable('gyro.x', 'FP16')
        self._lg_stab.add_variable('gyro.y', 'FP16')
        self._lg_stab.add_variable('gyro.z', 'FP16')

        try:
            self._cf.log.add_config(self._lg_stab)
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration, {} not found in TOC'.format(str(e)))

        # ---------- 卡车2: 电机与IMU日志 ----------
        self._lg_motor = LogConfig(name='Motor', period_in_ms=ControlTime_in_ms)
        self._lg_motor.add_variable('stabilizer.thrust', 'uint16_t')
        self._lg_motor.add_variable('motor.m1', 'uint16_t')
        self._lg_motor.add_variable('motor.m2', 'uint16_t')
        self._lg_motor.add_variable('motor.m3', 'uint16_t')
        self._lg_motor.add_variable('motor.m4', 'uint16_t')
        self._lg_motor.add_variable('acc.x', 'FP16')
        self._lg_motor.add_variable('acc.y', 'FP16')
        self._lg_motor.add_variable('acc.z', 'FP16')

        # 挂载 estimator.rtRej
        try:
            self._lg_motor.add_variable('estimator.rtRej', 'uint32_t')
            self.has_rej_log = True
        except KeyError:
            self.has_rej_log = False

        try:
            self._cf.log.add_config(self._lg_motor)
            self._lg_motor.data_received_cb.add_callback(self._motor_log_data)
            self._lg_motor.error_cb.add_callback(self._motor_log_error)
            self._lg_motor.start()
        except KeyError as e:
            print('Could not start log configuration, {} not found in TOC'.format(str(e)))

        # ---------- 卡车3: 控制器日志 ----------
        self._lg_ctrl = LogConfig(name='Controller', period_in_ms=ControlTime_in_ms)
        self._lg_ctrl.add_variable('controller.roll', 'FP16')
        self._lg_ctrl.add_variable('controller.pitch', 'FP16')
        self._lg_ctrl.add_variable('controller.yaw', 'FP16')
        self._lg_ctrl.add_variable('posCtl.targetVX', 'FP16')
        self._lg_ctrl.add_variable('posCtl.targetVY', 'FP16')
        self._lg_ctrl.add_variable('posCtl.targetVZ', 'FP16')
        self._lg_ctrl.add_variable('pm.vbat', 'FP16')

        try:
            self._cf.log.add_config(self._lg_ctrl)
            self._lg_ctrl.data_received_cb.add_callback(self._ctrl_log_data)
            self._lg_ctrl.error_cb.add_callback(self._ctrl_log_error)
            self._lg_ctrl.start()
        except KeyError as e:
            print('Could not start log configuration, {} not found in TOC'.format(str(e)))

        # 安全关机定时器
        t = Timer(control_whole_time, self._clean_shutdown)
        t.start()

        self._cf.commander.send_setpoint(0, 0, 0, 0)

    def _stab_log_data(self, timestamp, data, logconf):
        if self.is_shutting_down:
            return

        # ================= WX-PID: 更新当前状态对象 AXis6f =================
        self.state.x = data['stateEstimate.x']
        self.state.y = data['stateEstimate.y']
        self.state.z = data['stateEstimate.z']

        self.state.vx = data['stateEstimate.vx']
        self.state.vy = data['stateEstimate.vy']
        self.state.vz = data['stateEstimate.vz']

        self.state.roll = data['stabilizer.roll']
        self.state.pitch = data['stabilizer.pitch']
        self.state.yaw = data['stabilizer.yaw']

        # 更新历史数组用于画图
        state_information[self.i, 0] = self.state.x
        state_information[self.i, 1] = self.state.y
        state_information[self.i, 2] = self.state.z
        state_information[self.i, 3] = self.state.roll
        state_information[self.i, 4] = self.state.pitch
        state_information[self.i, 5] = self.state.yaw
        state_information[self.i, 6] = self.state.vx
        state_information[self.i, 7] = self.state.vy
        state_information[self.i, 8] = self.state.vz
        state_information[self.i, 9] = data['gyro.x']
        state_information[self.i, 10] = data['gyro.y']
        state_information[self.i, 11] = data['gyro.z']

        # 调用核心控制器
        self._main_flight_controller()

        if self.i < data_len - 2:
            self.i += 1

    def _main_flight_controller(self):
    # 外部 WX-PID 核心控制器与状态机

        yawrate = 0

        # ---------- 设定目标轨迹 (Set-point) ----------
        if self.i < hover_step_len:
            # 1. 在低高度悬停准备
            x_offset = 0.0
            y_offset = 0.0
            z = low_z
            mode = 'hover'

        elif self.i < hover_step_len + fig8_step_len:
            # 2. 带有角度旋转的三维八字形 (边画八字边爬升)
            t_traj = (self.i - hover_step_len) / Control_freq
            progress = (self.i - hover_step_len) / fig8_step_len

            # 基础水平八字形方程
            x_base = fig8_A * np.sin(omega * t_traj)
            y_base = fig8_B * np.sin(2 * omega * t_traj)

            # 坐标系旋转 (顺时针旋转指定角度)
            x_rot = x_base * cos_rot - y_base * sin_rot
            y_rot = x_base * sin_rot + y_base * cos_rot

            x_offset = x_rot
            y_offset = -y_rot  # 适配坐标系 Y 轴方向
            # 高度随进度从 low_z 线性增加到 high_z
            z = low_z + progress * (high_z - low_z)
            mode = 'figure8_ascent'

        elif self.i < hover_step_len + fig8_step_len + return_step_len:
            # 3. 轨迹结束，逐渐缩回原点，并将高度过渡到 stable_z
            t_end = fig8_step_len / Control_freq

            # 计算八字形阶段终点坐标，用于平滑过渡
            x_base_end = fig8_A * np.sin(omega * t_end)
            y_base_end = fig8_B * np.sin(2 * omega * t_end)
            x_rot_end = x_base_end * cos_rot - y_base_end * sin_rot
            y_rot_end = x_base_end * sin_rot + y_base_end * cos_rot

            return_progress = (self.i - hover_step_len - fig8_step_len) / max(return_step_len - 1, 1)
            return_progress = np.clip(return_progress, 0.0, 1.0)

            # 坐标线性衰减回 0
            x_offset = (1.0 - return_progress) * x_rot_end
            y_offset = -(1.0 - return_progress) * y_rot_end
            # 高度平滑调整至稳定的着陆准备高度
            z = high_z + return_progress * (stable_z - high_z)
            mode = 'return_origin'

        else:
            # 4. 原点上空 stable_z 高度稳定
            x_offset = 0.0
            y_offset = 0.0
            z = stable_z
            mode = 'stable'


        # 组装目标点对象
        self.set_point.x = state_information[0, 0] + x_offset
        self.set_point.y = state_information[0, 1] - y_offset
        self.set_point.z = z

        target_pos_information[self.i] = np.array([self.set_point.x, self.set_point.y, self.set_point.z])

        # ---------- 调用 WX-PID 控制算法 ----------
        thrust, pitch, roll = self.control.positionController(self.state, self.set_point)

        target_alt_information[self.i] = np.array([roll, pitch, yawrate])
        target_vel_information[self.i] = np.array([self.control.setpoint_velocity.x,
                                                   self.control.setpoint_velocity.y,
                                                   self.control.setpoint_velocity.z])
        control_information[self.i, 0] = thrust

        # ---------- 下发指令: 状态机分段控制 ----------
        if mode in ['stable']:
            self._cf.commander.send_position_setpoint(self.set_point.x, self.set_point.y, self.set_point.z, 0)
        elif mode in ['hover', 'figure8_ascent', 'return_origin']:
            self._cf.commander.send_setpoint(roll, -pitch, yawrate, int(thrust))

        # ---------- 调试打印 ----------
        if self.i % 10 == 0:
            x_err = target_pos_information[self.i, 0] - state_information[self.i, 0]
            y_err = target_pos_information[self.i, 1] - state_information[self.i, 1]
            z_err = target_pos_information[self.i, 2] - state_information[self.i, 2]

            rej_str = f" | 拒绝: {self.current_rej_count}" if getattr(self, 'has_rej_log', False) else ""

            print(f"步数: {self.i:4d} | 时间: {self.i / Control_freq:5.4f} s | "
                  f"期望: ({target_pos_information[self.i, 0]:5.4f}, {target_pos_information[self.i, 1]:5.4f}, {target_pos_information[self.i, 2]:5.4f}) | "
                  f"实际: ({state_information[self.i, 0]:5.4f}, {state_information[self.i, 1]:5.4f}, {state_information[self.i, 2]:5.4f}) | "
                  f"误差: ({x_err:5.4f}, {y_err:5.4f}, {z_err:5.4f}){rej_str}")

    @staticmethod
    def _stab_log_error(logconf, msg):
        print('Error when Stabilizer logging %s: %s' % (logconf.name, msg))

    def _motor_log_data(self, timestamp, data, logconf):
        if self.i < data_len:
            control_information[self.i, 0] = data['stabilizer.thrust']
            control_information[self.i, 1] = data['motor.m1']
            control_information[self.i, 2] = data['motor.m2']
            control_information[self.i, 3] = data['motor.m3']
            control_information[self.i, 4] = data['motor.m4']
            acc_information[self.i, 0] = data['acc.x']
            acc_information[self.i, 1] = data['acc.y']
            acc_information[self.i, 2] = data['acc.z']

            # 更新拒绝监控状态
            if getattr(self, 'has_rej_log', False):
                self.current_rej_count = data.get('estimator.rtRej', 0)

    @staticmethod
    def _motor_log_error(logconf, msg):
        print('Error when Motor logging %s: %s' % (logconf.name, msg))

    def _ctrl_log_data(self, timestamp, data, logconf):
        if self.i < data_len:
            target_alt_information[self.i, 0] = data['controller.roll']
            target_alt_information[self.i, 1] = data['controller.pitch']
            target_alt_information[self.i, 2] = data['controller.yaw']
            target_vel_information[self.i, 0] = data['posCtl.targetVX']
            target_vel_information[self.i, 1] = data['posCtl.targetVY']
            target_vel_information[self.i, 2] = data['posCtl.targetVZ']
            battery_information[self.i, 0] = data['pm.vbat']

    @staticmethod
    def _ctrl_log_error(logconf, msg):
        print('Error when Ctrl logging %s: %s' % (logconf.name, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self.is_connected = False
        self._cf.commander.send_setpoint(0, 0, 0, 0)

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    @staticmethod
    def _connection_lost(link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _clean_shutdown(self):
        print("\n正在执行安全清理与断开连接 (降落确认中，请耐心等待3秒)...")
        self.is_shutting_down = True

        for _ in range(30):
            self._cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)

        self._cf.commander.send_notify_setpoint_stop()

        if hasattr(self, '_lg_stab'):
            self._lg_stab.stop()
            self._lg_stab.delete()
        if hasattr(self, '_lg_motor'):
            self._lg_motor.stop()
            self._lg_motor.delete()
        if hasattr(self, '_lg_ctrl'):
            self._lg_ctrl.stop()
            self._lg_ctrl.delete()

        self._cf.close_link()
        print("清理完成，已安全断开连接。下次无需重启！\n")


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # ================= 启动 Nokov 并监听 ================
    if ret == 0:
        print("动捕系统连接成功，已开启实时位置注入...")
        nokov_client.PySetDataCallback(nokov_to_cf_callback, None)
    else:
        print(f"动捕系统连接失败 (错误码:{ret})，请检查网络或软件设置！")
        exit(0)

    le = CrazyflieTest(uri)

    print(f"项目名称: Quadrotor——{os.path.basename(__file__)}")
    print(f"时间步长: {ControlTime_in_ms} s")
    print(f"总飞行步数: {step} 步")
    print(f"总飞行时间: {ControlTime_in_ms * step / 1000.0} s")
    print("=" * 100)

    while le.is_connected:
        time.sleep(0.1)

    plot_len = max(1, min(le.i, data_len - 1))

    plot_data_for_record = {
        'time': np.arange(plot_len) / Control_freq,
        'dt'            : np.full(plot_len, dt),
        'Control_freq'  : np.full(plot_len, Control_freq),
        'rotor_radius'  : np.full(plot_len, R),
        'rotor_diameter': np.full(plot_len, D),
        'mass'          : np.full(plot_len, mass),
        'x'        : state_information[:plot_len, 0] - state_information[0, 0],
        'y'        : state_information[:plot_len, 1] - state_information[0, 1],
        'z'        : state_information[:plot_len, 2],
        'phi'      : state_information[:plot_len, 3],
        'theta'    : state_information[:plot_len, 4],
        'psi'      : state_information[:plot_len, 5],
        'v_x'      : state_information[:plot_len, 6],
        'v_y'      : state_information[:plot_len, 7],
        'v_z'      : state_information[:plot_len, 8],
        'v_phi'    : state_information[:plot_len, 9],
        'v_theta'  : state_information[:plot_len, 10],
        'v_psi'    : state_information[:plot_len, 11],
        'height'   : state_information[:plot_len, 2],
        'heightR'  : state_information[:plot_len, 2] / R,
        'heightD'  : state_information[:plot_len, 2] / D,
        'x_tar'    : target_pos_information[:plot_len, 0] - state_information[0, 0],
        'y_tar'    : target_pos_information[:plot_len, 1] - state_information[0, 1],
        'z_tar'    : target_pos_information[:plot_len, 2],
        'phi_tar'  : target_alt_information[:plot_len, 0],
        'theta_tar': target_alt_information[:plot_len, 1],
        'psi_tar'  : target_alt_information[:plot_len, 2],
        'v_x_tar'  : target_vel_information[:plot_len, 0],
        'v_y_tar'  : target_vel_information[:plot_len, 1],
        'v_z_tar'  : target_vel_information[:plot_len, 2],
        'T'        : control_information[:plot_len, 0],
        'm1'       : control_information[:plot_len, 1],
        'm2'       : control_information[:plot_len, 2],
        'm3'       : control_information[:plot_len, 3],
        'm4'       : control_information[:plot_len, 4],
        'acc_x'    : acc_information[:plot_len, 0],
        'acc_y'    : acc_information[:plot_len, 1],
        'acc_z'    : acc_information[:plot_len, 2],
        'V_bat'    : battery_information[:plot_len, 0],
        'Sensor'   : np.full(plot_len, 'Nokov'),
        'err_x'    : target_pos_information[:plot_len, 0] - state_information[:plot_len, 0],
        'err_y'    : target_pos_information[:plot_len, 1] - state_information[:plot_len, 1],
        'err_z'    : target_pos_information[:plot_len, 2] - state_information[:plot_len, 2],
        'err_v_x'  : target_vel_information[:plot_len, 0] - state_information[:plot_len, 6],
        'err_v_y'  : target_vel_information[:plot_len, 1] - state_information[:plot_len, 7],
        'err_v_z'  : target_vel_information[:plot_len, 2] - state_information[:plot_len, 8]
    }

    print("=" * 100)
    print("生成结果图表...")

    plotter.create_comprehensive_plot(plot_data_for_record)
    plt.show()

    filename_record = f'Crazyflie_(PID)_data_Figure8_{int(rotation_angle_deg)}.csv'
    logger.save_to_csv(plot_data_for_record, filename_record)
    print(f"全量实验数据已保存至: {filename_record}")