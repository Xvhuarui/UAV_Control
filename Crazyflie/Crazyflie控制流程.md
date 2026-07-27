<div align='center'>
        <h1>Crazyflie控制流程
</div>

# 1. 硬件准备与连接

## 1.1 硬件准备

1）Crazyflie 2.1 无人机：无人机实验本体

2）CrazyRadio PA：与无人机连接并通讯

3）地面站电脑：作为地面站用来控制无人机

4）Crazyflie Mark Deck 配 4个 动捕球：用于动捕系统进行位置捕捉

5）电池充电器配 7块 电池：对多个电池进行充电，保证无人机实验的循环进行

6）电子秤：对不同电池与无人机的组合进行质量测量

7）动捕加密狗：用于动捕软件的使用

## 1.2 硬件连接

&emsp;&emsp;动捕数据通过网线与地面站电脑连接，通过设置IP地址实现数据传输。之后地面站电脑通过CrazyRadio PA与无人机连接，将动捕数据以及轨迹位置信息发送给无人机，无人机根据数据进行控制。

<div align=center>
    <img src="./Assets/Crazyflie硬件连接.png" width=80% style="margin-top: 20px; margin-bottom: 30px;">
</div>

&emsp;&emsp;在设置中找到以太网并设置“编辑IP分配”为手动，将“IP地址”设置为10.1.1.198，”子网掩码“设置为255.255.255.0，即可与动捕系统进行通讯。（注：该设置在后续有线网络连接时会连接不上，只需将“编辑IP分配”设置为自动即可。）

# 2. 程序讲解

## 2.1 程序用途

<div align=center>

| 程序名称 | 程序位置 | 控制器 | 程序用途 |
| :-----: | :-----: | :----: | :-----: |
|  01_base_Crazyflie(WX-PID_Nokov_Hover).py  | 主目录 | PID控制程序 | 用于控制Crazyflie的悬停运动 |
|  01_base_Crazyflie(WX-PID_Nokov_SameVDifferentz).py | 主目录 | PID控制程序 | 用于控制Crazyflie的相同速度不同高度运动 |
|  01_base_Crazyflie(WX-PID_Nokov_Archimedes).py  | 主目录 | PID控制程序 | 用于控制Crazyflie的阿基米德螺旋线运动 |
|  01_base_Crazyflie(WX-PID_Nokov_Figure8).py  | 主目录 | PID控制程序 | 用于控制Crazyflie的8字飞行运动 |
|  WX_PID_Controller.py | 主目录/Controller | 无 | 为王旭师兄撰写的PID控制器函数 |
|  utils_data.py | 主目录/Utilities | 无 | 用于数据记录的工具函数 |
|  utils_plot.py | 主目录/Utilities | 无 | 用于数据可视化的工具函数 |

</div>

