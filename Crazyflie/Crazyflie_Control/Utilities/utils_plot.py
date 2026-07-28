import numpy as np
import matplotlib.pyplot as plt


class UAVPlotter:
    def __init__(self):
        # 设置中文字体，兼容 Windows (SimHei) 和 Mac (Arial Unicode MS)
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS']
        plt.rcParams['axes.unicode_minus'] = False

    def create_comprehensive_plot(self, data):
        """创建综合绘图 (16宫格完美布局)"""
        fig = plt.figure(figsize=(20, 16))

        # 第一行：位置(x,y,z)与 X速度
        self._plot_position_tracking(fig, data)

        # 第二行：姿态(phi,theta,psi)与 Y速度
        self._plot_attitude_tracking(fig, data)

        # 第三行：动力系统(推力, 电机, 角速度)与 Z速度
        self._plot_controls_and_z_vel(fig, data)

        # 第四行：平面与3D轨迹
        self._plot_trajectories(fig, data)

        # 调整子图间距
        plt.tight_layout(pad=3.0)
        return fig

    def _plot_position_tracking(self, fig, data):
        """第一行：绘制位置跟踪图 (1, 2, 3) + X速度 (4)"""
        ax1 = plt.subplot(4, 4, 1)
        self._plot_tracking(ax1, data['time'], data['x'], data['x_tar'], 'x位置跟踪', 'x位置 (m)',
                            ['实际x位置', '目标x位置'])

        ax2 = plt.subplot(4, 4, 2)
        self._plot_tracking(ax2, data['time'], data['y'], data['y_tar'], 'y位置跟踪', 'y位置 (m)',
                            ['实际y位置', '目标y位置'])

        ax3 = plt.subplot(4, 4, 3)
        self._plot_tracking(ax3, data['time'], data['z'], data['z_tar'], 'z位置跟踪', 'z位置 (m)',
                            ['实际z位置', '目标z位置'])

        ax4 = plt.subplot(4, 4, 4)
        self._plot_tracking(ax4, data['time'], data['v_x'], data['v_x_tar'], 'x速度跟踪', 'x速度 (m/s)',
                            ['实际x速度', '期望x速度'])

    def _plot_attitude_tracking(self, fig, data):
        """第二行：绘制姿态跟踪图 (5, 6, 7) + Y速度 (8)"""
        ax5 = plt.subplot(4, 4, 5)
        self._plot_tracking(ax5, data['time'], data['phi'], data['phi_tar'], 'phi(滚转)跟踪', '角度 (度)',
                            ['实际滚转角', '目标滚转角'])

        ax6 = plt.subplot(4, 4, 6)
        self._plot_tracking(ax6, data['time'], data['theta'], data['theta_tar'], 'theta(俯仰)跟踪', '角度 (度)',
                            ['实际俯仰角', '目标俯仰角'])

        ax7 = plt.subplot(4, 4, 7)
        self._plot_tracking(ax7, data['time'], data['psi'], data['psi_tar'], 'psi(偏航)跟踪', '角度 (度)',
                            ['实际偏航角', '目标偏航角'])

        ax8 = plt.subplot(4, 4, 8)
        self._plot_tracking(ax8, data['time'], data['v_y'], data['v_y_tar'], 'y速度跟踪', 'y速度 (m/s)',
                            ['实际y速度', '期望y速度'])

    def _plot_controls_and_z_vel(self, fig, data):
        """第三行：绘制动力系统与角速度图 (9, 10, 11) + Z速度 (12)"""
        # ax9: 总推力指令
        ax9 = plt.subplot(4, 4, 9)
        ax9.plot(data['time'], data['T'], label='总推力指令', linewidth=1.5, color='red')
        self._format_plot(ax9, '总推力指令', '时间 (s)', '推力 (PWM)')

        # ax10: 四个电机推力
        ax10 = plt.subplot(4, 4, 10)
        ax10.plot(data['time'], data['m1'], label='Motor 1', linewidth=1, alpha=0.8)
        ax10.plot(data['time'], data['m2'], label='Motor 2', linewidth=1, alpha=0.8)
        ax10.plot(data['time'], data['m3'], label='Motor 3', linewidth=1, alpha=0.8)
        ax10.plot(data['time'], data['m4'], label='Motor 4', linewidth=1, alpha=0.8)
        self._format_plot(ax10, '四电机实际推力', '时间 (s)', '推力 (PWM)')

        # ax11: 三轴实际角速度
        ax11 = plt.subplot(4, 4, 11)
        ax11.plot(data['time'], data['v_phi'], label='滚转角速度(wx)', linewidth=1.5, color='blue')
        ax11.plot(data['time'], data['v_theta'], label='俯仰角速度(wy)', linewidth=1.5, color='green')
        ax11.plot(data['time'], data['v_psi'], label='偏航角速度(wz)', linewidth=1.5, color='purple')
        self._format_plot(ax11, '三轴实际角速度', '时间 (s)', '角速度 (deg/s)')

        # ax12: Z速度跟踪
        ax12 = plt.subplot(4, 4, 12)
        self._plot_tracking(ax12, data['time'], data['v_z'], data['v_z_tar'], 'z速度跟踪', 'z速度 (m/s)',
                            ['实际z速度', '期望z速度'])

    def _plot_trajectories(self, fig, data):
        """第四行：绘制2D与3D轨迹图 (13, 14, 15, 16)"""
        ax13 = plt.subplot(4, 4, 13)
        self._plot_2d_trajectory(ax13, data['x'], data['y'], data['x_tar'], data['y_tar'], 'XY 平面轨迹', 'x位置 (m)',
                                 'y位置 (m)')

        ax14 = plt.subplot(4, 4, 14)
        self._plot_2d_trajectory(ax14, data['x'], data['z'], data['x_tar'], data['z_tar'], 'XZ 平面轨迹', 'x位置 (m)',
                                 'z位置 (m)')

        ax15 = plt.subplot(4, 4, 15)
        self._plot_2d_trajectory(ax15, data['y'], data['z'], data['y_tar'], data['z_tar'], 'YZ 平面轨迹', 'y位置 (m)',
                                 'z位置 (m)')

        ax16 = plt.subplot(4, 4, 16, projection='3d')
        ax16.plot(data['x'], data['y'], data['z'], label='实际轨迹', linewidth=2, alpha=0.8)
        ax16.plot(data['x_tar'], data['y_tar'], data['z_tar'], '--', label='目标轨迹', linewidth=2, alpha=0.8)
        ax16.set_xlabel('x位置 (m)', fontsize=12)
        ax16.set_ylabel('y位置 (m)', fontsize=12)
        ax16.set_zlabel('z位置 (m)', fontsize=12)
        ax16.set_title('三维轨迹跟踪', fontsize=14, fontweight='bold')
        ax16.legend(fontsize=10)
        ax16.grid(True, alpha=0.3)

    def _plot_tracking(self, ax, time, actual, target, title, ylabel, labels):
        """绘制跟踪曲线通用函数"""
        ax.plot(time, actual, label=labels[0], linewidth=1.5)
        ax.plot(time, target, '--', label=labels[1], linewidth=1.5)
        self._format_plot(ax, title, '时间 (s)', ylabel)

    def _plot_2d_trajectory(self, ax, x_actual, y_actual, x_target, y_target, title, xlabel, ylabel):
        """绘制2D轨迹"""
        ax.plot(x_actual, y_actual, label='实际轨迹', linewidth=2, alpha=0.8)
        ax.plot(x_target, y_target, '--', label='目标轨迹', linewidth=2, alpha=0.8)
        self._format_plot(ax, title, xlabel, ylabel)
        ax.axis('equal')

    def _format_plot(self, ax, title, xlabel, ylabel):
        """格式化绘图"""
        ax.set_xlabel(xlabel, fontsize=12)
        ax.set_ylabel(ylabel, fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
