<div align="center">
  <h1 align="center"> quest2 VR </h1>
  <h3 align="center"> Agilex Robotics </h3>
  <p align="center">
    <a href="README.md"> English </a> | <a>中文</a> 
  </p>
</div>



## 介绍

该仓库实现了使用 meta quest2 VR 套装对 piper 机械臂进行遥操作。

### 准备工作 

将代码克隆到 ROS 下的工作空间：

```bash
git clone https://github.com/RoboPPN/questVR.git   #quest2遥操作代码

git clone git@github.com:agilexrobotics/lifting_ws.git   #升降杆设备代码

git clone git@github.com:agilexrobotics/Piper_ros.git  	  #piper机械臂代码
```

我们在 Ubuntu 20.04 上测试了我们的代码，其他操作系统可能需要不同的配置。

有关更多信息，您可以参考 [開始使用 Meta Quest 2](https://www.meta.com/zh-tw/help/quest/articles/getting-started/getting-started-with-quest-2/?srsltid=AfmBOoqvDcwTtPt2P9o6y3qdXT_9zxz4m8yyej4uwLGEXVXv6KAr3QQz) 、[Piper_ros](https://github.com/agilexrobotics/Piper_ros)、[oculus_reader](https://github.com/rail-berkeley/oculus_reader)、 [lifting_ws](https://github.com/agilexrobotics/lifting_ws)。


### oculus_reader

该存储库提供了从 Quest2 读取位置和按下按钮的工具。

以VR眼镜作为基站，将手柄的与基站的TF关系传输给机械臂。

## 软件启动

将上述克隆下来的三份代码放在同一个工作空间进行编译，随后在`~/catkin_ws/src/oculus_reader/scripts`目录下对代码文件进行修改。

1、如果你需要遥操作单臂，就修改`pinocchio_vr_single_piper.py`，如果你是遥操作双臂的，就选择`pinocchio_vr_double_piper.py`进行修改。

需要修改的内容为将代码文件中的`urdf_path`的值修改成piper的urdf路径即可。

2、对于`oculus_control_double_piper.py`以及`oculus_control_single_piper.py`文件中的 0.263、-0.263 ，该值为左右机械臂安装在原点的左边的0.263米与-0.263米。根据你的机械臂安装位置填写即可。

### 启动

1、启动机械臂

将左右机械臂的can口名绑定为left_piper、right_piper，并在piper的驱动包里面新建一个launch文件用于开启两个机械臂。launch文件内容如下：

```xml
<launch>
  <!-- 定义 mode 参数，默认值为 0,读取主从臂的消息，并转发到ROS -->
  <arg name="mode" default="1" />
  <arg name="auto_enable" default="true" />
  <!-- 启动左侧机械臂节点 -->
    <node name="$(anon piper_left)" pkg="piper" type="piper_start_ms_node.py" output="screen">
      <param name="can_port" value="left_piper" />
      <param name="mode" value="$(arg mode)" />
      <param name="auto_enable" value="$(arg auto_enable)" />
      <remap from="/puppet/joint_states" to="/puppet/joint_left" />
      <remap from="/master/joint_states" to="/left_joint_states" />
    </node>

  <!-- 启动右侧机械臂节点 -->
    <node name="$(anon piper_right)" pkg="piper" type="piper_start_ms_node.py" output="screen">
      <param name="can_port" value="right_piper" />
      <param name="mode" value="$(arg mode)" />
      <param name="auto_enable" value="$(arg auto_enable)" />
      <remap from="/puppet/joint_states" to="/puppet/joint_right" />
      <remap from="/master/joint_states" to="right_joint_states" />
    </node>
</launch>
```

如果只想控制一个机械臂，则激活can口后直接运行`start_single_piper.launch`文件即可。

2、启动升降杆

如果想启用升降杆的话

直接在lifting_ctrl功能包里直接运行

```bash
roslaunch lifting_ctrl start_motor.launch
```



3、启动VR遥操作

在VR眼镜里面进入USB调试后，运行：

```bash
roslaunch oculus_reader oculus_control_double_piper.launch   #双臂

roslaunch oculus_reader oculus_control_single_piper.launch   #单臂
```

















