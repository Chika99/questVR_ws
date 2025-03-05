<div align="center">
  <h1 align="center"> quest teleop piper </h1>
  <h3 align="center"> Agilex Robotics </h3>
  <p align="center">
    <a href="README.md"> English </a> | <a>中文</a> 
  </p>
</div>




## 介绍

该仓库实现了使用 meta quest2/3 VR 套装对 piper 机械臂进行遥操作。

### 准备工作 

1、安装依赖

```bash
conda create -n vt python=3.9

conda activate vt

conda install pinocchio -c conda-forge

pip install meshcat 

pip install casadi

sudo apt install android-tools-adb

```

2、配置您的 quest 设备

- quest必须支持开发者模式

  如在 quest 设备上找不到开发者模式，请参考该链接配置 quest 设备。

  点击：[oculus_reader](https://github.com/rail-berkeley/oculus_reader)

- 在quest上安装alvr_client_android.apk文件，文件在代码里面的oculus_reader/apk目录下。

  安装教程参考下面 [meta quest 安装 apk 的方法] 章节内容。

3、将代码克隆到 ROS 下的工作空间：

```bash
git clone https://github.com/RoboPPN/questVR.git
```

我们在 Ubuntu 20.04 上测试了我们的代码，其他操作系统可能需要不同的配置。

有关更多信息，您可以参考 [開始使用 Meta Quest 2](https://www.meta.com/zh-tw/help/quest/articles/getting-started/getting-started-with-quest-2/?srsltid=AfmBOoqvDcwTtPt2P9o6y3qdXT_9zxz4m8yyej4uwLGEXVXv6KAr3QQz) 、[Piper_ros](https://github.com/agilexrobotics/Piper_ros)、[oculus_reader](https://github.com/rail-berkeley/oculus_reader)。

4、进入到 piper_description 功能包下，将 urdf 目录下的 piper_description.urdf 所有 STL 文件的用户名修改成你的用户名，例如：

```xml
<geometry>
        <mesh
          filename="/home/agilex/questVR_ws/src/Piper_ros/src/piper_description/meshes/base_link.STL" />
</geometry>

修改成

<geometry>
        <mesh
          filename="/home/<your name>/questVR_ws/src/Piper_ros/src/piper_description/meshes/base_link.STL" />
</geometry>
```

### 代码架构说明

oculus_reader，该存储库提供了从 Quest 设备读取位置和按下按钮的工具。

以VR眼镜作为基站，将手柄的与基站的TF关系传输给机械臂。

```bash
├── APK    #apk文件
│   ├── alvr_client_android.apk
│   └── teleop-debug.apk
├── CMakeLists.txt
├── config
│   └── oculus_reader.rviz
├── launch	#启动文件
│   ├── teleop_double_piper.launch
│   └── teleop_single_piper.launch
├── package.xml
└── scripts
    ├── buttons_parser.py
    ├── FPS_counter.py
    ├── install.py
    ├── oculus_reader.py
    ├── piper_control.py		#机械臂控制接口
    ├── teleop_double_piper.py	#遥操作双臂代码
    ├── teleop_single_piper.py	#遥操作单臂代码
    └── tools.py
```

## meta quest 安装 apk 的方法

[步骤1] 到meta商店安装Mobile VR Station 这个应用

[步骤2] 把要安装的android apk 档案放到根目录Dowanload 里面(meta浏览器预设的下载目录) ，影片里建议直接安装apkpure 商店( [https://m.apkpure.com/tw/)](https://ref.gamer.com.tw/redir.php?url=https%3A%2F%2Fm.apkpure.com%2Ftw%2F))，即可之后都从apkpure 商店里面安装其他热门android 应用

[步骤3] 开启Mobile VR Station => Configuration Wizard => Show All Options => Configuration Scoped Storage => Step1: Request Access => 选择根目录Dowanload 里面刚刚步骤2放的apk 点两下进行安装

## 软件启动

将上述克隆下来进行编译。

### 启动

1、使能机械臂can模块

参考该库的2.1小结说明 [piper_ros](https://github.com/agilexrobotics/Piper_ros)

- 如果是跑单臂的话将can端口名设置为can0

- 双臂的话就设置为left_piper、right_piper

2、启动遥操机械臂

```bash
roslaunch oculus_reader teleop_single_piper.launch    # 单臂遥操

roslaunch oculus_reader teleop_double_piper.launch    # 双臂遥操
```

## 操作说明

- 遥操单臂使用右手手柄，开始遥操前确保机械臂回到初始姿态，按住按键 “A” 能使机械臂回到初始位置，长按按键 “B” 为遥操机械臂，松开为停止控制。遥操双臂同理。  

- 为了操作的人身安全以及减少对机械臂的损害，在遥操结束后，请确保机械臂回到初始位置附件再按下“A”||“X”键复位。













