# Tonybot — 竞赛机器人代码

本仓库包含 Tonybot 机器人两套方案的代码：主控（ESP32）和视觉协处理器（ESP32-S3），使用 **PlatformIO + VS Code** 开发。

## 仓库结构

```
├── Esp32/                    # [主控] PlatformIO 项目
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
├── Esp32S3/                  # [视觉] PlatformIO 项目
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
├── PillarNavigator/          # [保留] 同学 Arduino IDE 原版代码
├── ESP32S3Cam/               # [保留] 同学 Arduino IDE 原版代码
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
- **ESP32-S3** (视觉): 摄像头、AI 物体识别、HSV 颜色检测，通过 I2C 把结果发给 ESP32

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

## 关键模块说明

### 主控 (ESP32)

- **TumbleGuard / FallDetector**: 两套独立的跌倒检测。均使用 IMU 角度判断，跌倒时调用动作组起立
- **LidarDriver**: A1M8 RPLIDAR 驱动，含形状识别（区分球体/正方体/圆柱体）
- **HeadTracker**: 头部舵机追踪颜色目标
- **ObjectFollower**: 整体跟随颜色目标
- **CirclePillar**: 绕柱导航

### 视觉 (ESP32-S3)

- I2C 从机地址 `0x52`
- 寄存器: `0x00`=红色, `0x01`=绿色, `0x02`=蓝色 色块数据; `0x10`=AI 标签结果; `0x20`=AI 形状结果
- `update_ai_iic_data()` 更新 AI 结果到 I2C 缓存
