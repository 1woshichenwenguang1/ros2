# Carla车辆键盘遥操作（任务1，Carla模拟器）

任务 1：物理仿真与键盘运动控制，仿真器为 CARLA（不使用 Gazebo）。

## 运行环境
- Windows 宿主机：CARLA 服务端 0.9.13（CarlaUE4.exe，端口 2000）。
- 课程虚拟机（Ubuntu 20.04、ROS Noetic）：carla Python 客户端 0.9.13（已预装）。
- 宿主机 IP：虚拟机经 NAT 访问宿主机，取 VMnet8 网关地址，默认 192.168.27.1（宿主机 ipconfig 核实）。

## 架构与控制原理
- main.py 直连 CARLA Python API：连接重试并打印服务端版本；世界设为同步模式，固定步长 0.05 秒；生成 tesla.model3 自车与 640x360 RGB 相机；键盘循环读取 w/a/s/d/x；每步打印速度与油门/转向/刹车/倒挡状态。
- vehicle_control.py 为纯映射逻辑：`s` 键在 |v| < 0.5 m/s 时挂倒挡并给半油门，高速时先刹车；`x` 为全刹停车；不依赖 carla 包，可离线单测。
- 无窗口取证模式（--headless --demo）：虚拟机无 3D 加速时自动巡航并把相机帧逐帧存为 PNG（run_evidence/），作为可运行性证据。

## 运行步骤
1. Windows 启动 CarlaUE4.exe。
2. 虚拟机交互运行：
   `python3 src/carla_vehicle_keyboard_teleop/scripts/main.py --host 192.168.27.1 --town Town03`
   终端按 w/a/s/d/x 驾驶，q 退出。
3. 无窗口取证：
   `python3 src/carla_vehicle_keyboard_teleop/scripts/main.py --host 192.168.27.1 --headless --demo --frames 120`
4. launch 启动：
   `roslaunch src/carla_vehicle_keyboard_teleop/launch/main.launch host:=192.168.27.1`
   无窗口：加 `headless:=true`。

## 接口
- 控制输出：`carla.VehicleControl(throttle, steer, brake, reverse)`，每个同步步施加一次。
- 感知输入：RGB 相机帧（无窗口模式存 PNG 证据）。
- ROS 话题：本模块走纯 CARLA API，不依赖话题。

## 测试
`python3 -m unittest tests.test_carla_keyboard_local`
覆盖倒挡规则、刹车油门映射与入口文件存在性，不需要 CARLA 服务端。

## 运行效果
![headless evidence](../../img/ground/carla_vehicle_keyboard_teleop/evidence.gif)
