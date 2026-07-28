<div align='center'>
        <h1>Crazyflie控制流程
</div>

# 1. 硬件准备与连接

## 1.1 硬件准备

以下硬件可分为Crazyflie组、动捕组、配件组。

其中Crazyflie组包括：

-   Crazyflie 2.1 无人机：无人机实验本体
-   CrazyRadio PA：与无人机连接并通讯
-   地面站电脑：作为地面站用来控制无人机
-   电池充电器配 7块 电池：对多个电池进行充电，保证无人机实验的循环进行

其中动捕组包括：

-   Crazyflie Mark Deck 配 4个 动捕球：用于动捕系统进行位置捕捉 
-   动捕加密狗：用于动捕软件的使用

其中配件组包括：

-   电子秤：对不同电池与无人机的组合进行质量测量

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
| :------ | :------ | :----: | :------ |
|  01_base_Crazyflie(WX-PID_Nokov_Hover).py  | 主目录 | PID控制程序 | 用于控制Crazyflie的悬停运动 |
|  01_base_Crazyflie(WX-PID_Nokov_SameVDifferentz).py | 主目录 | PID控制程序 | 用于控制Crazyflie的相同速度不同高度运动 |
|  01_base_Crazyflie(WX-PID_Nokov_Archimedes).py  | 主目录 | PID控制程序 | 用于控制Crazyflie的阿基米德螺旋线运动 |
|  01_base_Crazyflie(WX-PID_Nokov_Figure8).py  | 主目录 | PID控制程序 | 用于控制Crazyflie的8字飞行运动 |
|  WX_PID_Controller.py | 主目录/Controller | 无 | 为王旭师兄撰写的PID控制器函数 |
|  utils_data.py | 主目录/Utilities | 无 | 用于数据记录的工具函数 |
|  utils_plot.py | 主目录/Utilities | 无 | 用于数据可视化的工具函数 |

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

```python
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
```

### 2.2.3 动捕函数使用

```python
if ret == 0:
        print("动捕系统连接成功，已开启实时位置注入...")
        nokov_client.PySetDataCallback(nokov_to_cf_callback, None)
    else:
        print(f"动捕系统连接失败 (错误码:{ret})，请检查网络或软件设置！")
        exit(0)
```

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
