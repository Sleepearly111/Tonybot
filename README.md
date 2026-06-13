# 双柱绕行 - Tonybot 竞赛代码

## 项目简介

本项目包含 Tonybot 机器人视觉导航功能的三套方案：

### ESP32S3Cam
ESP32-S3 视觉模块程序。负责摄像头图像采集、HSV 颜色空间转换、目标颜色识别，并通过 IIC 将检测结果发送给主控板。

### PillarNavigator
主控板双柱绕行导航程序。接收视觉模块的颜色数据，控制舵机和运动实现在两根彩色柱子之间的导航：
- 目标跟随接近（ObjectFollower）
- 头部追踪（HeadTracker）
- 绕柱避障（CirclePillar）
- 跌倒检测（FallDetector）

### EndZoneClassify
终点区 LiDAR 导航 + 物体识别程序。使用 RPLidar 激光雷达进行环境扫描，通过物体检测和聚类算法找到置物台上的三个目标物体，控制机器人导航到终点区，并在到达后识别物体形状（球体/正方体/圆柱体）。

**导航策略（navigate2，当前使用）：**
1. 等待雷达连续 2 次扫描 mid 角度稳定（波动 < 8°）
2. 转向对准中间物体（mid 在 90° ± 10°）
3. 对准后直走逼近，不做方向修正
   - 距离 > 130cm → 每次前进 3 步
   - 距离 ≤ 130cm → 每次前进 1 步
4. 到达终点区（距离 65~95cm）后，连续 3 次分类一致输出结果

**硬件接线：**
| 组件 | 接口 | 引脚 |
|------|------|------|
| RPLidar 雷达 | Serial1 | RX=GPIO32, TX=GPIO33 |
| 雷达电机 | GPIO | GPIO13 |
| 舵机控制器 | Serial2 | RX=GPIO16, TX=GPIO17 |

## 硬件平台

- 主控：ESP32
- 视觉模块：ESP32-S3 CAM
- 舵机：Lobot 串行总线舵机
- IMU：QMI8658

## 目录结构

```
├── ESP32S3Cam/              # ESP32-S3 视觉模块（摄像头+颜色检测+IIC发送）
├── PillarNavigator/         # 主控双柱绕行导航程序
│   ├── src/
│   └── pack/
├── EndZoneClassify/         # 终点区 LiDAR 导航 + 物体识别
│   ├── EndZoneClassify.ino  # 主程序
│   ├── lidar_common.*       # 雷达驱动 + 数据采集
│   ├── lidar_navigate2.*    # 导航方案2（当前使用）
│   ├── lidar_action.*       # 舵机动作执行
│   ├── lidar_classify.*     # 物体形状识别
│   ├── lidar_endzone.*      # 终点区定位
│   ├── RPLidar.*            # RoboPeak 雷达库
│   ├── inc/                 # 雷达协议头文件
│   └── src/                 # 机器人底层驱动
└── libraries/               # 外部依赖库
```

## 开发环境

- Arduino IDE + ESP32 板级支持包
- ESP32-S3 编译链
- 依赖库见 `libraries/` 目录，需复制到 Arduino `libraries` 文件夹
