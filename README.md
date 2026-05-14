# 双柱绕行 - Tonybot 竞赛代码

## 项目简介

本项目包含 Tonybot 机器人视觉导航功能的两套方案：

### ESP32S3Cam
ESP32-S3 视觉模块程序。负责摄像头图像采集、HSV 颜色空间转换、目标颜色识别，并通过 IIC 将检测结果发送给主控板。

### PillarNavigator
主控板双柱绕行导航程序。接收视觉模块的颜色数据，控制舵机和运动实现在两根彩色柱子之间的导航：
- 目标跟随接近（ObjectFollower）
- 头部追踪（HeadTracker）
- 绕柱避障（CirclePillar）
- 跌倒检测（FallDetector）

## 硬件平台

- 主控：ESP32
- 视觉模块：ESP32-S3 CAM
- 舵机：Lobot 串行总线舵机
- IMU：QMI8658

## 目录结构

```
├── ESP32S3Cam/          # ESP32-S3 视觉模块（摄像头+颜色检测+IIC发送）
├── PillarNavigator/       # 主控双柱绕行导航程序
│   ├── src/               # 本地依赖库
│   │   ├── BUZZER/
│   │   ├── IMU/
│   │   ├── IR/
│   │   ├── LobotServoCtl/
│   │   ├── PwmServo/
│   │   ├── Sensor/
│   │   └── WMMatrixLed/
│   └── pack/              # 参考安装包
└── libraries/             # 外部依赖库（QMI8658、SensorLib）
```

## 开发环境

- Arduino IDE + ESP32 板级支持包
- ESP32-S3 编译链
- 依赖库见 `libraries/` 目录，需复制到 Arduino `libraries` 文件夹
