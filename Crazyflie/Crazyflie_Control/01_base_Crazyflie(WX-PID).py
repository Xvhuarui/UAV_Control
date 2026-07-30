# -*- coding: utf-8 -*-
"""
Created on Fri Dec 13 15:30:42 2024
@author: Wang Xu

Recreated on Sat Wed 21 22:11:19 2026
@author: XHR
"""

import logging  # 导入日志模块
import time  # 导入时间模块
from threading import Timer, Thread  # 导入定时器模块

import numpy as np
import matplotlib.pyplot as plt
import os

from Controller.WX_PID_Controller import Position_controller, AXis6f

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
logging.basicConfig(level=logging.ERROR)

from Utilities.utils_plot import UAVPlotter
from Utilities.utils_data import DataLoggerIdeal

plotter = UAVPlotter()
logger = DataLoggerIdeal(output_dir='Plot_&_Data')

class CrazyflieTest:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True
        self.i = 0
        self.is_shutting_down = False
        
        # PID控制相关类的初始化
        self.control = Position_controller(Control_freq)
        self.state = AXis6f()
        self.set_point = AXis6f()

    def _connected(self, link_uri):
        """
        连接成功后的回调函数。
        注意：这里不能直接设参数和死等，必须开辟新线程去执行！
        """
        print('Successfully connected to %s' % link_uri)

        # 启动一个独立的初始化线程
        Thread(target=self._initialization_sequence).start()

    def _initialization_sequence(self):
        """
        独立线程：负责重置滤波器 -> 配置并启动日志 -> 启动安全定时器
        """
        # ================== 0. 重置卡尔曼滤波器 ==================
        print("正在重置内部状态估计器，请保持无人机静止...")
        self._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(1.5)  # 这里的 sleep 非常安全，因为它在独立线程里
        print("重置完成，准备起飞！")
        print("=" * 100)

        # ================== 1. 创建并启动卡车1日志 ==================
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

        # ================== 2. 创建并启动卡车2日志 ==================
        self._lg_motor = LogConfig(name='Motor', period_in_ms=ControlTime_in_ms)
        self._lg_motor.add_variable('motor.m1', 'uint16_t')
        self._lg_motor.add_variable('motor.m2', 'uint16_t')
        self._lg_motor.add_variable('motor.m3', 'uint16_t')
        self._lg_motor.add_variable('motor.m4', 'uint16_t')
        self._lg_motor.add_variable('motion.squal', 'uint8_t')
        self._lg_motor.add_variable('motion.shutter', 'uint16_t')

        try:
            self._cf.log.add_config(self._lg_motor)
            self._lg_motor.data_received_cb.add_callback(self._motor_log_data)
            self._lg_motor.error_cb.add_callback(self._motor_log_error)
            self._lg_motor.start()
        except KeyError as e:
            print('Could not start log configuration, {} not found in TOC'.format(str(e)))

        # ================== 3. 启动控制与安全机制 ==================
        # 注意：这里调用的是我们上一轮写的 _clean_shutdown 清理函数
        t = Timer(control_whole_time, self._clean_shutdown)
        t.start()

        self._cf.commander.send_setpoint(0, 0, 0, 0)

    def _stab_log_data(self, timestamp, data, logconf):
        """
        Callback from the log API when data arrives
        当接收到无人机按设定频率下发的一组日志数据时，自动触发此回调函数。
        其中，timestamp、logconf为标志位，
        """
        if self.is_shutting_down:
            return

        # ----------------- 3. 更新当前状态对象 -----------------
        # 将字典 data 里的原始数据，提取并赋值给 self.state 实例。
        # 这是为了把格式整理好，方便下一步直接扔给 PID 控制器去计算
        self.state.x = data['stateEstimate.x']
        self.state.y = data['stateEstimate.y']
        self.state.z = data['stateEstimate.z']

        self.state.vx = data['stateEstimate.vx']
        self.state.vy = data['stateEstimate.vy']
        self.state.vz = data['stateEstimate.vz']

        self.state.roll = data['stabilizer.roll']
        self.state.pitch = data['stabilizer.pitch']
        self.state.yaw = data['stabilizer.yaw']

        # 记录实际状态数据到绘图矩阵
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

        self._main_flight_controller()

        # 安全机制：防止数组越界。只要还没跑到设定的总时间，就把记录行数 self.i 加 1
        if self.i < data_len - 2:
            self.i = self.i + 1

    def _main_flight_controller(self):
        """核心控制器：轨迹生成、PID运算与指令下发"""
        yawrate = 0  # 偏航角速度默认设为0

        # ----------------- 1. 设定目标轨迹与状态机分段 -----------------
        if self.i < 20:
            # 【阶段 0：破除安全锁】前 0.2 秒
            x, y, z = state_information[0, 0], state_information[0, 1], 0.0
            mode = 'unlock_wait'

        elif self.i < 50:
            # 【阶段 1：起飞过渡期】
            x = state_information[0, 0]
            y = state_information[0, 1]
            z = 0.1
            mode = 'internal_position'

        elif self.i < (data_len - end_pos_len):
            # 【阶段 2：主体飞行期】(外部 PID 接管)
            x = 0
            y = 0
            z = 0.5
            mode = 'external_pid'

        else:
            # 【阶段 3：降落收尾期】
            last_idx = data_len - end_pos_len - 1
            x = state_information[last_idx, 0]
            y = state_information[last_idx, 1]
            z = 0.2
            mode = 'internal_landing'

        # ----------------- 2. 组装目标点对象 -----------------
        self.set_point.x = state_information[0, 0] + x
        self.set_point.y = state_information[0, 1] - y
        self.set_point.z = z + 0.0

        target_pos_information[self.i] = np.array([self.set_point.x, self.set_point.y, self.set_point.z])

        # ----------------- 3. 【核心运算】调用 PID 控制算法 -----------------
        thrust, pitch, roll = self.control.positionController(self.state, self.set_point)

        target_alt_information[self.i] = np.array([roll, pitch, yawrate])
        target_vel_information[self.i] = np.array([self.control.setpoint_velocity.x,
                                                   self.control.setpoint_velocity.y,
                                                   self.control.setpoint_velocity.z])
        control_information[self.i, 0] = thrust

        # ----------------- 4. 【下发指令】根据当前模式执行 -----------------
        if mode == 'unlock_wait':
            # 死命发送 0 推力解锁电机
            self._cf.commander.send_setpoint(0, 0, 0, 0)
        elif mode == 'internal_position':
            self._cf.commander.send_position_setpoint(state_information[0, 0],
                                                      state_information[0, 1], 0.1, 0)
        elif mode == 'external_pid':
            # 外部 PID 控制，pitch 取反适配
            self._cf.commander.send_setpoint(roll, -pitch, yawrate, int(thrust))
        elif mode == 'internal_landing':
            last_idx = data_len - end_pos_len - 1
            self._cf.commander.send_position_setpoint(state_information[last_idx, 0],
                                                      state_information[last_idx, 1], 0.2, 0)

        # ----------------- 5. 状态更新与调试输出 -----------------
        if self.i % 10 == 0:
            x_err = target_pos_information[self.i, 0] - state_information[self.i, 0]
            y_err = target_pos_information[self.i, 1] - state_information[self.i, 1]
            z_err = target_pos_information[self.i, 2] - state_information[self.i, 2]

            print(f"步数: {self.i:4d} | 时间: {self.i / Control_freq:5.2f} s | "
                  f"期望位置: ({target_pos_information[self.i, 0]:5.2f}, {target_pos_information[self.i, 1]:5.2f}, {target_pos_information[self.i, 2]:5.2f}) | "
                  f"实际位置: ({state_information[self.i, 0]:5.2f}, {state_information[self.i, 1]:5.2f}, {state_information[self.i, 2]:5.2f}) | "
                  f"位置误差: ({x_err:5.2f}, {y_err:5.2f}, {z_err:5.2f}) | "
                  f"光流质量：({flow_information[self.i, 0]}, {flow_information[self.i, 1]})")

    @staticmethod
    def _stab_log_error(logconf, msg):
        """
        Callback from the log API when an error occurs
        当日志 API 发生错误时，底层会自动调用这个回调函数。

        参数说明：
        - logconf: 发生错误的那个日志配置块对象（也就是你在 _connected 里实例化的那个 _lg_stab）。
        - msg: 底层传回来的具体错误信息字符串。
        """

        # 使用旧式的字符串格式化 (%s)，将出问题的日志块名称（比如 'Stabilizer'）和具体的错误原因打印到终端。
        # 这样你在看黑框框（控制台）时，就能立刻知道是哪一批数据出了问题，以及为什么没传过来。
        print('Error when Stabilizer logging %s: %s' % (logconf.name, msg))

    def _motor_log_data(self, timestamp, data, logconf):
        if self.i < data_len:
            control_information[self.i, 1] = data['motor.m1']
            control_information[self.i, 2] = data['motor.m2']
            control_information[self.i, 3] = data['motor.m3']
            control_information[self.i, 4] = data['motor.m4']

            flow_information[self.i, 0] = data['motion.squal']
            flow_information[self.i, 1] = data['motion.shutter']

    @staticmethod
    def _motor_log_error(logconf, msg):
        """
        Callback from the log API when an error occurs
        当日志 API 发生错误时，底层会自动调用这个回调函数。

        参数说明：
        - logconf: 发生错误的那个日志配置块对象（也就是你在 _connected 里实例化的那个 _lg_stab）。
        - msg: 底层传回来的具体错误信息字符串。
        """

        # 使用旧式的字符串格式化 (%s)，将出问题的日志块名称（比如 'Stabilizer'）和具体的错误原因打印到终端。
        # 这样你在看黑框框（控制台）时，就能立刻知道是哪一批数据出了问题，以及为什么没传过来。
        print('Error when Motor logging %s: %s' % (logconf.name, msg))

    def _disconnected(self, link_uri):
        """
        Callback when the Crazyflie is disconnected (called in all cases)
        当 Crazyflie 的连接被彻底断开时（无论是因为正常关闭还是异常掉线，所有情况下都会兜底调用此函数）。
        """

        # 1. 打印断开提示，告诉你这次飞行通信彻底结束了
        print('Disconnected from %s' % link_uri)

        # 2. 【关键状态切换】解除主程序的死循环
        # 把标志位设为 False，这样最下方主程序里的 `while le.is_connected: time.sleep(0.1)` 就会立刻结束。
        # 程序才能顺理成章地往下走，去执行你写的画图 (UAVPlotter) 和保存数据 (DataLoggerIdeal) 的代码。
        self.is_connected = False

        # 3. 【终极安全锁】强制电机停转
        # 发送 0 滚转、0 俯仰、0 偏航角速度、0 推力的指令。
        # 这是一种极为严谨的“防暴冲”机制。哪怕程序马上就要退出了，也要在最后一口气告诉无人机：“立刻切断所有动力！”
        # 防止无人机在失去控制前记住的是最后一个带有大推力的指令，导致它自己往天花板上撞。
        self._cf.commander.send_setpoint(0, 0, 0, 0)

    def _connection_failed(self, link_uri, msg):
        """
        Callback when connection initial connection fails
        当初始连接尝试彻底失败时（例如指定的地址找不到 Crazyflie），触发此回调。
        """

        # 在控制台打印出连接失败的地址和具体原因，方便你排查硬件或参数问题
        print('Connection to %s failed: %s' % (link_uri, msg))

        # 【关键状态切换】将连接状态标志位强行设为 False
        # 这一步极其重要！因为在你的主程序 if __name__ == '__main__': 里面，
        # 有一个死循环 `while le.is_connected: time.sleep(0.1)` 正在傻傻地等。
        # 如果不把这个标志位改成 False，连接失败后你的 Python 脚本就会永远卡在那里不退出。
        self.is_connected = False

    @staticmethod
    def _connection_lost(link_uri, msg):
        """
        Callback when disconnected after a connection has been made (i.e Crazyflie moves out of range)
        当无人机已经成功建立连接后，由于意外原因（如飞出遥控范围、电池突然掉电、遇到强信号干扰）导致连接断开时，底层 API 会自动触发此回调。
        """

        # 在控制台打印出断连的地址和具体原因，提醒操作者无人机失控了
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _clean_shutdown(self):
        print("\n正在执行安全清理与断开连接 (降落确认中，请耐心等待3秒)...")

        # 1. 触发标志位：强行挂起日志回调，不准它再干扰我们关机
        self.is_shutting_down = True

        # 2. 终极解锁与降落确认：连续发送 0 推力
        # 必须循环发送至少 3 秒，让无人机系统判定已经“安全着陆”
        # 否则断开瞬间会被当做坠机，导致下次飞行死锁
        for _ in range(30):
            self._cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)

        # 3. 官方停机宣告
        self._cf.commander.send_notify_setpoint_stop()

        # 4. 停止并释放日志内存
        if hasattr(self, '_lg_stab'):
            self._lg_stab.stop()
            self._lg_stab.delete()
        if hasattr(self, '_lg_motor'):
            self._lg_motor.stop()
            self._lg_motor.delete()

        # 5. 断开物理连接
        self._cf.close_link()
        print("清理完成，已安全断开连接。下次无需重启！\n")
        

if __name__ == '__main__':
    # Initialize the low-level drivers
    
    ControlTime_in_ms = 10  # 时间步长
    step = 1000  # 总飞行步数
    Control_freq = 1000.0 / ControlTime_in_ms  # 控制频率
    end_pos_len = 100  # 控制结束时间

    flight_time = (ControlTime_in_ms * step) / 1000.0
    control_whole_time = flight_time + 1

    cflib.crtp.init_drivers()

    data_len = int(control_whole_time * Control_freq)
    state_information      = np.zeros([data_len, 12])
    target_pos_information = np.zeros([data_len, 3])
    target_alt_information = np.zeros([data_len, 3])
    target_vel_information = np.zeros([data_len, 3])
    control_information    = np.zeros([data_len, 5])
    flow_information       = np.zeros([data_len, 2])

    le = CrazyflieTest(uri)

    print(f"项目名称: Quadrotor——{os.path.basename(__file__)}")
    print(f"时间步长: {ControlTime_in_ms} s")
    print(f"总飞行步数: {step} 步")
    print(f"总飞行时间: {ControlTime_in_ms * step / 1000.0} s")
    print("=" * 100)

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(0.1)

    plot_len = data_len-1
    plot_data = {
        'time': np.arange(plot_len) / Control_freq,
        'x': state_information[:plot_len, 0] - state_information[0, 0],
        'y': state_information[:plot_len, 1] - state_information[0, 1],
        'z': state_information[:plot_len, 2],
        'phi': state_information[:plot_len, 3],  # 实际 Roll
        'theta': state_information[:plot_len, 4],  # 实际 Pitch
        'psi': state_information[:plot_len, 5],  # 实际 Yaw
        'v_x': state_information[:plot_len, 6],
        'v_y': state_information[:plot_len, 7],
        'v_z': state_information[:plot_len, 8],
        'v_phi'  : state_information[:plot_len, 9],   # 实际角速度 Roll Rate
        'v_theta': state_information[:plot_len, 10],  # 实际角速度 Pitch Rate
        'v_psi'  : state_information[:plot_len, 11],  # 实际角速度 Yaw Rate

        'x_tar': target_pos_information[:plot_len, 0] - state_information[0, 0],
        'y_tar': target_pos_information[:plot_len, 1] - state_information[0, 1],
        'z_tar': target_pos_information[:plot_len, 2],

        'phi_tar': target_alt_information[:plot_len, 0],  # 目标 Roll
        'theta_tar'  : target_alt_information[:plot_len, 1],  # 目标 Pitch
        'psi_tar'  : target_alt_information[:plot_len, 2],  # 目标 Yaw

        'v_x_tar': target_vel_information[:plot_len, 0],  # 目标速度 X
        'v_y_tar': target_vel_information[:plot_len, 1],  # 目标速度 Y
        'v_z_tar': target_vel_information[:plot_len, 2],  # 目标速度 Z
        
        'T' : control_information[:plot_len, 0],  # 总推力 PWM
        'm1': control_information[:plot_len, 1],
        'm2': control_information[:plot_len, 2],
        'm3': control_information[:plot_len, 3],
        'm4': control_information[:plot_len, 4]

    }
    # =============================== 结果可视化 ===============================
    print("=" * 100)
    print("生成结果图表...")

    plotter.create_comprehensive_plot(plot_data)
    plt.show()

    # =============================== 保存数据文件 ===============================
    filename = 'Crazyflie_(WX_PID)_data_test1.P-DNN_csv'
    logger.save_to_csv(plot_data, filename)
