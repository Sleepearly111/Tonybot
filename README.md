# 头部颜色追踪 - Tonybot 竞赛代码

## 项目简介

本项目包含 Tonybot 机器人头部颜色追踪功能的两套方案：

### ESP32S3Cam_ColorDetection
ESP32-S3 摄像头模块上的颜色检测程序。负责摄像头图像采集、HSV 颜色空间转换、目标颜色识别，并通过 IIC 将检测结果发送给主控板。

### color_follow
主控板的颜色追踪与头部运动控制程序。接收摄像头颜色数据，控制舵机实现头部对目标的实时追踪，同时支持：
- 头部追踪（HeadTracker）
- 物体跟随（ObjectFollower）
- 圆形柱体检测（CirclePillar）
- 跌倒检测（FallDetector）

## 硬件平台

- 主控：ESP32
- 视觉模块：ESP32-S3 CAM
- 舵机：Lobot 串行总线舵机
- IMU：QMI8658

## 目录结构

```
├── ESP32S3Cam_ColorDetection/   # ESP32-S3 摄像头颜色检测
├── color_follow/                # 头部颜色追踪主控程序
│   ├── src/                     # 本地依赖库
│   │   ├── BUZZER/
│   │   ├── IMU/
│   │   ├── IR/
│   │   ├── LobotServoCtl/
│   │   ├── PwmServo/
│   │   ├── Sensor/
│   │   └── WMMatrixLed/
│   └── pack/                    # 参考安装包
└── libraries/                   # 外部依赖库
```

## 开发环境

- Arduino IDE + ESP32 板级支持包
- ESP32-S3 编译链
- 依赖库见 `libraries/` 目录，需复制到 Arduino `libraries` 文件夹
