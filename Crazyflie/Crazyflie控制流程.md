<div align='center'>
        <h1>Crazyflie控制流程
</div>

# 1. 硬件准备与连接

## 1.1 硬件准备

以下硬件可分为Crazyflie组、动捕组、配件组。

其中Crazyflie组包括：

-   Crazyflie 2.1 无人机：无人机实验本体，以下简写为Crazyflie
-   CrazyRadio PA：与Crazyflie连接并通讯
-   地面站电脑：作为地面站用来控制Crazyflie
-   电池充电器配 7块 电池：对多个电池进行充电，保证Crazyflie实验的循环进行

其中动捕组包括：

-   Crazyflie Mark Deck 配 4个 动捕球：用于动捕系统进行位置捕捉 
-   动捕加密狗：用于动捕软件的使用

其中配件组包括：

-   电子秤：对不同电池与Crazyflie的组合进行质量测量

## 1.2 硬件连接

&emsp;&emsp;动捕数据通过网线与地面站电脑连接，通过设置IP地址实现数据传输。之后地面站电脑通过CrazyRadio PA与Crazyflie连接，将动捕数据以及轨迹位置信息发送给Crazyflie，Crazyflie根据数据进行控制。

<div align=center>
    <img src="./Assets/Crazyflie硬件连接.png" width=80% style="margin-top: 20px; margin-bottom: 30px;">
</div>

&emsp;&emsp;在设置中找到以太网并设置“编辑IP分配”为手动，将“IP地址”设置为10.1.1.198，”子网掩码“设置为255.255.255.0，即可与动捕系统进行通讯，并关闭防火墙和网络拦截软件。（注：该设置在后续外部网络使用时会连接错误，只需将“编辑IP分配”设置为自动即可。）

# 2. 程序讲解

## 2.1 程序用途

<div align=center>

<table>
  <thead>
    <tr>
      <th align="center">程序名称</th>
      <th align="center">程序位置</th>
      <th align="center">控制器</th>
      <th align="center">程序用途</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="left"><a href="./Crazyflie_Control/01_base_Crazyflie(WX-PID_Nokov_Hover).py">01_base_Crazyflie(WX-PID_Nokov_Hover).py</a></td>
      <td align="left">主目录</td>
      <td align="center">PID控制程序</td>
      <td align="left">用于控制Crazyflie的悬停运动</td>
    </tr>
    <tr>
      <td align="left"><a href="./Crazyflie_Control/01_base_Crazyflie(WX-PID_Nokov_SameVDifferentz).py">01_base_Crazyflie(WX-PID_Nokov_SameVDifferentz).py</a></td>
      <td align="left">主目录</td>
      <td align="center">PID控制程序</td>
      <td align="left">用于控制Crazyflie的相同速度不同高度运动</td>
    </tr>
    <tr>
      <td align="left"><a href="./Crazyflie_Control/01_base_Crazyflie(WX-PID_Nokov_Archimedes).py">01_base_Crazyflie(WX-PID_Nokov_Archimedes).py</a></td>
      <td align="left">主目录</td>
      <td align="center">PID控制程序</td>
      <td align="left">用于控制Crazyflie的阿基米德螺旋线运动</td>
    </tr>
    <tr>
      <td align="left"><a href="./Crazyflie_Control/01_base_Crazyflie(WX-PID_Nokov_Figure8).py">01_base_Crazyflie(WX-PID_Nokov_Figure8).py</a></td>
      <td align="left">主目录</td>
      <td align="center">PID控制程序</td>
      <td align="left">用于控制Crazyflie的8字飞行运动</td>
    </tr>
    <tr>
      <td align="left"><a href="./Crazyflie_Control/Controller/WX_PID_Controller.py">WX_PID_Controller.py</a></td>
      <td align="left">主目录/Controller</td>
      <td align="center">/</td>
      <td align="left">为王旭师兄撰写的PID控制器函数</td>
    </tr>
    <tr>
      <td align="left"><a href="./Crazyflie_Control/Utilities/utils_data.py">utils_data.py</a></td>
      <td align="left">主目录/Utilities</td>
      <td align="center">/</td>
      <td align="left">用于数据记录的工具函数</td>
    </tr>
    <tr>
      <td align="left"><a href="./Crazyflie_Control/Utilities/utils_plot.py">utils_plot.py</a></td>
      <td align="left">主目录/Utilities</td>
      <td align="center">/</td>
      <td align="left">用于数据可视化的工具函数</td>
    </tr>
  </tbody>
</table>

</div>

## 2.2 动捕涉及代码解析

### 2.2.1 动捕库导入并初始化

&emsp;&emsp;以下代码旨在初始化动捕系统的通信客户端。首先导入 ```nokov.nokovsdk``` 模块，创建 ```PySDKClient``` 实例并赋值给变量 ```nokov_client```；然后调用其 ```Initialize()``` 方法，传入动捕服务器 IP 地址（转换为字节串），该方法返回一个表示初始化结果的状态码（或布尔值），赋值给 ```ret``` 变量，以便后续判断连接是否成功。

```python
from nokov.nokovsdk import *

nokovIp = '10.1.1.198'
nokov_client = PySDKClient()
ret = nokov_client.Initialize(bytes(nokovIp, encoding="utf8"))
```

### 2.2.2 动捕函数设置

&emsp;&emsp;以下代码旨在对动捕数据进行处理，将其转换为Crazyflie可识别的位置信息。具体来说，该函数会从动捕数据中提取出刚体的位置信息，并将其转换为米为单位的位置信息，同时进行坐标系映射。然后，将该位置信息发送给Crazyflie，Crazyflie 飞控内部会对接收到的外部位置信息进行多传感器融合滤波（如 EKF），以输出更平滑的估计值。其中通过时间戳节流将发送频率限制在约 50Hz，目的是避免因数据发送过快导致无线电链路拥塞或缓冲区溢出。

```python
def nokov_to_cf_callback(pFrameOfMocapData, pUserData):
# 动捕数据接收回调：只注入位置(X, Y, Z)
    global global_cf, debug_counter, last_extpos_time
    if pFrameOfMocapData is None or global_cf is None:
        return

    # 频率节流 (限制在约50Hz，防止无线电缓冲区溢出)
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
```

### 2.2.3 动捕函数使用

&emsp;&emsp;以下代码旨在对运行上述动捕函数。但是在外部进行了状态判断，若初始化失败，则退出程序。若初始化成功，则开启实时位置注入。

```python
if ret == 0:
    print("动捕系统连接成功，已开启实时位置注入...")
    nokov_client.PySetDataCallback(nokov_to_cf_callback, None)
else:
    print(f"动捕系统连接失败 (错误码:{ret})，请检查网络或软件设置！")
    exit(0)
```

## 2.3 控制涉及代码解析

### 2.3.1 ```CrazyflieTest```类内部函数用途

&emsp;&emsp;该类内部包含了多个函数，用于控制Crazyflie的运动。具体来说，该类包含了以下函数：

&emsp;&emsp;（1）初始化组函数：

-   ```__init__(self, link_uri)```：初始化函数，用于初始化CrazyflieTest类，传入通讯地址作为参数

&emsp;&emsp;（2）通讯连接组函数：

-   ```_connected(self, link_uri)```：连接成功函数，作为标志位，用于处理Crazyflie通讯连接成功后的操作
-   ```_disconnected(self, link_uri)```：断开连接函数，作为标志位，用于处理Crazyflie通讯断开连接后的操作
-   ```_connection_failed(self, link_uri, msg)```：连接失败函数，作为标志位，用于处理Crazyflie通讯连接失败后的操作
-   ```_connection_lost(link_uri, msg)```（静态函数）：连接丢失函数，作为标志位，用于处理Crazyflie通讯连接丢失后的操作

&emsp;&emsp;（3）控制组函数：

-   ```_initialization_sequence(self)```：参数初始化函数，用于启动 EKF 多重滤波器，保证动捕数据的稳定接入
-   ```_main_flight_controller(self)```：主飞行控制器函数，在其中设置期望轨迹，同时调用PID控制器进行控制
-   ```_clean_shutdown(self)```：清理函数，在每次飞行结束后用于清理Crazyflie寄存器中的内容，进而重复飞行无需重启设备

&emsp;&emsp;（4）日志组函数：   

-   ```_stab_log_data(self, timestamp, data, logconf)```：状态日志函数，用于记录12维状态变量（位置、速度、姿态、角速度）信息
-   ```_stab_log_error(logconf, msg)```（静态函数）：状态日志报警函数，用于对状态日志函数的运行错误进行报警处理
-   ```_motor_log_data(self, timestamp, data, logconf)```：电机日志函数，用于记录整体升力、四电机升力与三轴加速度信息
-   ```_motor_log_error(logconf, msg)```（静态函数）：电机日志报警函数，用于对电机日志函数的运行错误进行报警处理
-   ```_ctrl_log_data(self, timestamp, data, logconf)```：控制日志函数，用于记录期望姿态、期望速度与电池电压信息
-   ```_ctrl_log_error(logconf, msg)```（静态函数）：控制日志报警函数，用于对控制日志函数的运行错误进行报警处理

### 2.3.2 主要控制轨迹

&emsp;&emsp;以下轨迹涉及相关参数均可直接从程序参数初始化部分进行修改：

&emsp;&emsp;（1）悬停运动轨迹

```python
if self.i < start_step_len:
    x_offset = 0.0
    y_offset = 0.0
    z = 0.04
    mode = 'takeoff'

elif self.i < (data_len - end_step_len):
    x_offset = 0.0
    y_offset = 0.0
    z = z_func
    mode = 'fly'

else:
    x_offset = 0.0
    y_offset = 0.0
    z = 0.04
    mode = 'land'
```

&emsp;&emsp;（2）相同速度不同高度运动轨迹

```python
if cycle_id < cycle_num:
    if cycle_step < hover_step_len:
        x_offset = 0.0
        y_offset = 0.0
        z = low_z
        mode = 'low_hover'

    elif cycle_step < hover_step_len + rise_step_len:
        # 以 vertical_speed 从 low_z 匀速上升到 high_z
        rise_progress = (cycle_step - hover_step_len) / max(rise_step_len - 1, 1)
        rise_progress = np.clip(rise_progress, 0.0, 1.0)

        x_offset = 0.0
        y_offset = 0.0
        z = low_z + rise_progress * (high_z - low_z)
        mode = 'rise'

    elif cycle_step < hover_step_len + rise_step_len + high_hover_step_len:
        # 悬停
        x_offset = 0.0
        y_offset = 0.0
        z = high_z
        mode = 'high_hover'

    else:
        # 以 vertical_speed 从 high_z 匀速下降到 low_z
        fall_progress = (cycle_step - hover_step_len - rise_step_len - high_hover_step_len) / max(fall_step_len - 1, 1)
        fall_progress = np.clip(fall_progress, 0.0, 1.0)

        x_offset = 0.0
        y_offset = 0.0
        z = high_z - fall_progress * (high_z - low_z)
        mode = 'fall'

else:
    x_offset = 0.0
    y_offset = 0.0
    z = low_z
    mode = 'final_low_hover'
```

&emsp;&emsp;（3）阿基米德螺旋线运动轨迹

```python
if self.i < hover_step_len:
    # 原点悬停
    x_offset = 0.0
    y_offset = 0.0
    z = z_func
    mode = 'hover'

elif self.i < hover_step_len + archimedes_step_len:
    # 阿基米德螺旋线
    t_traj = (self.i - hover_step_len) / Control_freq

    alpha = k1_func * np.pi / 4
    beta = k2_func * np.pi / 4

    current_r = r_growth * t_traj
    current_r = np.clip(current_r, 0.0, r_max)

    x_base = current_r * np.cos(omega * t_traj)
    y_base = -current_r * np.sin(omega * t_traj)

    x_rot = x_base * np.cos(beta) + y_base * np.sin(alpha) * np.sin(beta)
    y_rot = y_base * np.cos(alpha)
    z_rot = -x_base * np.sin(beta) + y_base * np.sin(alpha) * np.cos(beta)

    x_offset = x_rot
    y_offset = -y_rot
    z = z_func + z_rot
    mode = 'archimedes'

elif self.i < hover_step_len + archimedes_step_len + return_step_len:
    # 从阿基米德螺旋终点继续旋回原点，这里不是直线回原点，而是半径逐渐减小、角度继续旋转

    t_end = archimedes_step_len / Control_freq

    alpha = k1_func * np.pi / 4
    beta = k2_func * np.pi / 4

    # 螺旋结束时的半径和角度
    r_end = r_growth * t_end
    r_end = np.clip(r_end, 0.0, r_max)

    theta_end = omega * t_end

    # 回旋阶段时间
    t_return = (self.i - hover_step_len - archimedes_step_len) / Control_freq

    return_progress = (
        self.i - hover_step_len - archimedes_step_len) / max(return_step_len - 1, 1)
    return_progress = np.clip(return_progress, 0.0, 1.0)

    # 半径从 r_end 线性减小到 0
    current_r = (1.0 - return_progress) * r_end

    # 角度继续旋转，不中断
    theta = theta_end + omega * t_return

    x_base = current_r * np.cos(theta)
    y_base = -current_r * np.sin(theta)

    x_rot = x_base * np.cos(beta) + y_base * np.sin(alpha) * np.sin(beta)
    y_rot = y_base * np.cos(alpha)
    z_rot = -x_base * np.sin(beta) + y_base * np.sin(alpha) * np.cos(beta)

    x_offset = x_rot
    y_offset = -y_rot
    z = z_func + z_rot
    mode = 'spiral_return'

else:
    x_offset = 0.0
    y_offset = 0.0
    z = stable_z
    mode = 'stable'
```

&emsp;&emsp;（4）8字形运动轨迹

```python
if self.i < hover_step_len:
    # 在低高度悬停准备
    x_offset = 0.0
    y_offset = 0.0
    z = low_z
    mode = 'hover'

elif self.i < hover_step_len + fig8_step_len:
    # 带有角度旋转的三维八字形 (边画八字边爬升)
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
    # 轨迹结束，逐渐缩回原点，并将高度过渡到 stable_z
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
    # 原点上空 stable_z 高度稳定
    x_offset = 0.0
    y_offset = 0.0
    z = stable_z
    mode = 'stable'
```

# 3. 光流控制



# 附A：动捕库安装教程
&emsp;&emsp;将“nokovpy-3.0.1-py3-none-any.whl”文件放置于主目录下，并在终端执行以下命令进行动捕库的安装：

-   新安装：

    ```python
    pip install nokovpy-3.0.1-py3-none-any.whl
    ```

-   重装:

    ```python
    pip install --force-reinstall nokovpy-3.0.1-py3-none-any.whl
    ```
