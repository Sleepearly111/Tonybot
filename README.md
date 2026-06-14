# Tonybot — 竞赛机器人代码

CRAIC 2026 具身智能任务赛，双人协作开发。

| 角色 | 工具 | 负责 |
|------|------|------|
| **同学A** | VS Code + PlatformIO | `Esp32/`、`Esp32S3/`（主控+视觉）、`tools/`（调试工具） |
| **同学B** | Arduino IDE | `EndZoneClassify/`（LiDAR导航+识别）、`PillarNavigator/`、`ESP32S3Cam/`（已完成） |

## 仓库结构

```
├── Esp32/                    # [同学A] 主控 PlatformIO 项目
│   ├── src/                  # 源代码
│   │   ├── main.cpp          # 主程序入口
│   │   ├── globals.cpp       # 全局对象定义（Controller, hwsensor, sonarServo, imu）
│   │   ├── base_config.h     # 引脚定义（I2C/舵机/IMU 等）
│   │   ├── Hiwonder.hpp      # IMU 类声明
│   │   ├── LobotServoController.h  # 总线舵机控制
│   │   ├── Servo.h           # PWM 舵机抽象
│   │   ├── hw_esp32cam_ctl.* # ESP32Cam I2C 通信
│   │   │
│   │   ├── voice/            # 语音模块（TTS / ASR）
│   │   │   ├── HWSensor.*
│   │   │   └── ShapeVoicePlayer.*
│   │   ├── lidar/            # 激光雷达
│   │   │   └── LidarDriver.*
│   │   ├── fall/             # 跌倒检测
│   │   │   ├── TumbleGuard.*
│   │   │   └── FallDetector.*
│   │   ├── tracking/         # 目标追踪导航
│   │   │   ├── HeadTracker.*
│   │   │   ├── ObjectFollower.*
│   │   │   ├── CirclePillar.*
│   │   │   └── RobotTracking.*
│   │   ├── IMU/              # QMI8658 IMU 驱动 + MadgwickAHRS 滤波
│   │   ├── LobotServoCtl/    # 总线舵机协议实现
│   │   └── PwmServo/         # PWM 舵机实现
│   ├── lib/                  # 本地第三方库
│   │   ├── SensorLib/        # QMI8658 传感器库
│   │   ├── ArduTFLite/       # TensorFlow Lite
│   │   └── ...               # 其他依赖库
│   ├── platformio.ini        # 项目配置
│   └── fix_rplidar.py        # RPLIDAR 兼容性修复脚本
│
├── Esp32S3/                  # [同学A] 视觉 PlatformIO 项目
│   ├── src/
│   │   ├── main.cpp          # 主程序入口（AI 标签识别 + I2C 发送）
│   │   ├── camera_setting.*  # 摄像头配置
│   │   ├── VisionAI.*        # AI 视觉推理
│   │   ├── iic_data_send.*   # I2C 从机通信（寄存器 0x00~0x10）
│   │   ├── color_detection.* # HSV 颜色检测
│   │   └── who_ai_utils.*    # AI 工具函数
│   ├── lib/                  # 本地第三方库
│   ├── platformio.ini        # 项目配置
│   └── ...
│
├── PillarNavigator/          # [同学B] Arduino IDE 绕柱程序（已完成）
├── ESP32S3Cam/               # [同学B] Arduino IDE 视觉程序（已完成）
├── EndZoneClassify/           # [同学B] Arduino IDE LiDAR导航+物体识别（进行中）
├── LiDARTest/                # [同学B] Arduino IDE 雷达基础测试
├── tools/                    # [共享] PC端调试工具 (lidar_viewer.py)
└── README.md                 # 本文件
```

## 硬件架构

```
┌──────────────┐   I2C (0x52)   ┌──────────────────┐
│   ESP32-S3   │ ←───────────→ │     ESP32        │
│  (视觉协处理器) │  SDA=47,SCL=48  │   (主控制器)       │
│              │               │                  │
│  摄像头/Ov2640│               │ Serial2 → 舵机    │
│  AI推理/TFLite│               │ RPLIDAR A1M8     │
│  HSV颜色检测  │               │ IMU QMI8658      │
│              │               │ TTS语音合成模块    │
│              │               │ ASR语音识别模块    │
└──────────────┘               └──────────────────┘
```

- **ESP32** (主控): 舵机控制、激光雷达、IMU 跌倒检测、导航逻辑、TTS/ASR 语音
- **ESP32-S3** (视觉): 摄像头、**汉字标签分类**（球体/正方体/圆柱体）、HSV 颜色检测，通过 I2C 把结果发给 ESP32

### I2C 设备（GPIO22/23）

| 地址 | 设备 | 用途 |
|------|------|------|
| 0x52 | ESP32-S3 | 视觉协处理器 |
| 0x40 | TTS 模块 | 语音合成播报 |
| 0x34 | ASR 模块 | 语音指令识别 |
| (内部) | QMI8658 IMU | 六轴姿态检测 |

## 开发环境

### 前置要求

- [VS Code](https://code.visualstudio.com/)
- [PlatformIO IDE 插件](https://platformio.org/install/ide/install-vscode)

### 打开项目

在 VS Code 中：

1. 点击 **文件 → 打开文件夹** → 选择 `D:\Desktop\final\Esp32`（或 `Esp32S3`）
2. 等待 PlatformIO 自动加载（右下角状态栏出现 PlatformIO 图标）
3. VS Code 底部出现 PlatformIO 工具栏按钮

### VS Code 操作

| 操作 | 按钮位置 | 等效命令 |
|------|---------|---------|
| 编译 | 底部 `✓` (Build) | `pio run` |
| 上传 | 底部 `→` (Upload) | `pio run -t upload` |
| 串口监视器 | 底部 `🔌` (Serial Monitor) | `pio device monitor` |
| 清理 | 底部垃圾桶图标 | `pio run -t clean` |

选择项目：先打开对应项目的文件夹（Esp32 或 Esp32S3），PlatformIO 会自动识别配置。

> 也可在 VS Code 终端（`` Ctrl+` ``）中直接输入 `pio run` 等命令。

### Arduino IDE 原版代码

同学的 Arduino IDE 代码保留在 `PillarNavigator/` 和 `ESP32S3Cam/` 目录，未经修改。

## Git 协作流程

### 分支说明

- `main` — 稳定版本，比赛就绪代码
- `develop` — 日常开发分支
- `feat/*` — 新功能分支，从 `develop` 分出，完成后 PR 合回 `develop`

### 日常操作

```bash
# 1. 开始工作前，拉取最新代码
git pull origin develop

# 2. 查看当前状态
git status

# 3. 提交更改
git add <文件...>            # 添加指定文件，不要用 git add .
git commit -m "描述你的改动"

# 4. 推送到远程
git push origin develop
```

### 提交流程（多人协作）

```bash
# 1. 拉取最新 develop
git pull origin develop

# 2. 如果有冲突，解决后提交
git add <冲突文件>
git commit -m "解决冲突"

# 3. 推送
git push origin develop
```

### 注意事项

- **不要用 `git push --force`**，会覆盖别人的提交
- **不要用 `git add .`**，会把敏感文件或二进制文件一起提交
- 推送前先 `git pull`，避免冲突
- 如果 push 被拒绝，先 pull 再 push

## 硬件引脚 (base_config.h)

| 引脚 | 功能 |
|------|------|
| GPIO22/23 | I2C SDA/SCL |
| GPIO17/16 | Serial2 (舵机总线) |
| GPIO4 | 头部舵机 |
| GPIO32/33 | RPLIDAR A1M8 |
| GPIO13 | 雷达电机控制 |

## 比赛概述

CRAIC 2026 具身智能任务赛。机器人从起点出发自主完成 3 个连续任务（避障→识别→播报），全程无遥控。

### 场地布局

```
总宽 = 2100mm，总深 = 4400mm

起点区(600×500) → 通道一(500×500) → 避障区(2100×1300, 红蓝2根柱子)
  → 通道二(500×500) → 大终点区(600×500) → 空区(500×500) → 置物台(1800×400, 高300)

小起点/小终点: 200×200 居中于大区
所有 500 宽区域在 2100 内居中
避障区内: 2根圆柱障碍物(直径300，高300，红/蓝各一)
置物台上: 球体(φ200-300)、正方体(300³)、圆柱体(φ300×300)
标签: A4纸印"球体""正方体""圆柱体"，终点区展示用
标记区: 3个150×150框，距置物台200mm
```

### 任务流程

| 阶段 | 传感器 | 方法 |
|------|--------|------|
| 通道一穿越 | IMU + LiDAR | 航向保持 + 3物体星座居中 |
| 避障区绕柱 (Task 1) | 相机 | HeadTracker + ObjectFollower + CirclePillar（S型绕行，左右由裁判指定） |
| 找通道二入口 | LiDAR | 原地旋转扫描 3 物体方向 |
| 通道二穿越 | LiDAR | 3物体星座居中导航 |
| 终点区标签识别 | 相机(S3 AI) | 到终点区暂停 → S3识别A4纸汉字标签（球体/正方体/圆柱体） |
| 物体定位 (Task 2) | LiDAR | 根据标签结果，classifyObjectAt() 多次投票识别台上对应物体 |
| 标记区+播报 (Task 3) | LiDAR + TTS | 以物体为参照接近到标记区 → ShapeVoicePlayer 播报3次 |

核心思路：置物台上的 3 个物体贯穿全场始终可见，作为全程导航信标；终点区汉字标签决定识别目标。

## 关键模块说明

### 主控 (ESP32)

- **比赛状态机** (`main.cpp`): Task1(相机绕柱) → Task2(S3读汉字标签→LiDAR物体识别) → Task3(标记区+播报)，各5分钟超时保护
- **3物体信标导航**: LiDAR 检测3物体 → 计算中心偏移 → 修正方向，用于通道穿越和区域导航
- **TumbleGuard / FallDetector**: 两套独立的跌倒检测。均使用 IMU 角度判断，跌倒时调用动作组起立
- **LidarDriver**: A1M8 RPLIDAR 驱动，含形状识别（区分球体/正方体/圆柱体）
- **HeadTracker**: 头部舵机追踪颜色目标
- **ObjectFollower**: 整体跟随颜色目标
- **CirclePillar**: 绕柱导航

### 终点区导航 (EndZoneClassify, B 的 Arduino IDE 项目)

B 的 LiDAR 终点区导航项目，当前使用 **navigate2** 方案：

1. 雷达扫描 → 连续 2 次 mid 角度稳定（波动 < 8°）
2. 转向对准中间物体（mid 在 90° ± 10°）
3. 对准后直走逼近（> 130cm 每次 3 步，≤ 130cm 每次 1 步）
4. 到达终点区（距离 65~95cm）后，连续 3 次分类一致 → 输出结果

**硬件接线：** Serial1(RX=GPIO32, TX=GPIO33) 接雷达，GPIO13 接电机，Serial2(RX=GPIO16, TX=GPIO17) 接舵机。

```
EndZoneClassify/
├── EndZoneClassify.ino       # 主程序（navigate2 状态机）
├── lidar_common.*            # 雷达驱动 + g_map[360] 数据采集
├── lidar_navigate2.*         # 导航方案2（当前使用）
├── lidar_classify.*          # 物体形状识别（3轴分类）
├── lidar_endzone.*           # 终点区判定
├── lidar_action.*            # 舵机动作执行
├── RPLidar.* + inc/          # RoboPeak 雷达驱动
├── base_config.h             # 引脚定义
├── Hiwonder.hpp              # IMU
├── LobotServoController.h    # 总线舵机
├── Servo.h                   # PWM 舵机
├── src/                      # 机器人底层驱动
└── pack/                     # 第三方库
```

## 调试工具

### LiDAR 可视化 (`tools/lidar_viewer.py`) — 两人通用

PC 端实时雷达视图工具，接到串口数据就显示极坐标图。**同学A 和 同学B 都用同一个工具**。

```bash
pip install matplotlib pyserial numpy                    # 仅首次
python tools/lidar_viewer.py COM3                       # 实时显示
python tools/lidar_viewer.py COM3 --record              # 录制日志
python tools/lidar_viewer.py scan_20260611.log          # 回放日志
```

**同学A**：`main.cpp` 里 `#define LIDAR_STREAM_MODE` → `pio run -t upload` → 工具自动显示。

**同学B**：打开 `EndZoneClassify/EndZoneClassify.ino` → Arduino IDE 上传 → 同一工具自动显示。

数据协议统一为 `SCAN <ts> <count>\n<angle> <dist> <quality>\n... END`。

### 视觉 (ESP32-S3)

- I2C 从机地址 `0x52`
- 寄存器: `0x00`=红色, `0x01`=绿色, `0x02`=蓝色 色块数据; `0x10`=AI 标签结果; `0x20`=AI 形状结果
- `update_ai_iic_data()` 更新 AI 结果到 I2C 缓存
