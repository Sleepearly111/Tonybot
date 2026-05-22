# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

This is a **PlatformIO** project with two independent targets: `Esp32/` (main controller) and `Esp32S3/` (vision coprocessor).

```bash
# Build (open the project folder first, then run inside it)
pio run                          # compile
pio run -t clean                 # clean build cache
pio run -t upload                # build + upload to board
pio device monitor               # serial monitor (115200 baud)

# Test (test/ dirs are empty scaffolding — no tests yet)
pio test                         # run all tests
```

All commands run from inside the project directory (`Esp32/` or `Esp32S3/`). In VS Code, use the PlatformIO toolbar: **✓** = Build, **→** = Upload, **🔌** = Serial Monitor.

### Branch workflow

- `develop` — active development
- `main` — tested, competition-ready code
- New features → branch from `develop`, PR back to `develop`

## Current Status (2026-05-22)

**正在进行的任务：** `Esp32/src/main.cpp` 行走 + 形状识别集成测试

`main.cpp` 当前流程：
1. 初始化舵机（Serial2, pins 16/17）→ 直立（动作组 0）
2. 初始化激光雷达（Serial1, pins 32/33）→ 360° 扫描
3. 前进 2 步（动作组 25）→ 停止 → 前方 ±45° 形状识别（球/正方体/圆柱体）

**已知问题：**

| Issue | Severity | Status |
|-------|----------|--------|
| **TG1WDT_SYS_RESET** — `setup()` 中累计 `delay()` 超过 5 秒触发硬件看门狗复位 | 🔴 阻塞 | 未解决 |
| Serial1/Serial2 端口冲突已修复（雷达→Serial1，舵机→Serial2）| 🟢 已解决 | ✅ |
| LiDAR 坐标系映射：`HEADING_OFFSET=90`（雷达 90°=机器人 0°前方）| 🟢 已解决 | ✅ |
| 形状分类阈值调优 — 1° 分辨率下 >400mm 球体/圆柱体难区分 | 🟡 已知限制 | 待场地实测 |

**TG1WDT 已尝试方案：**
1. `disableCore1WDT()` → 报错 "Failed to remove Core 1 IDLE task from WDT"，无效
2. `wdt_config0.val = 0` 禁用硬件 WDT → 系统在 `lidar.begin()` 中彻底挂死
3. `LidarDriver.cpp` 中 `waitMs()` 定期喂狗 → 仍复位，因为 `main.cpp setup()` 里的 `delay(2000)` 没换成喂狗

**推荐修复方向：** 将 `main.cpp setup()` 中所有长 `delay()` 也换成 `waitMs()` 喂狗循环，或把初始化流程拆成非阻塞状态机。

---

## Project Context

**第二十八届中国机器人及人工智能大赛 - 具身智能任务赛（线下家庭服务场景）**。机器人从起点出发自主完成 3 个连续任务（避障→识别→播报），全程无遥控。

**Hardware:** ESP32 (main controller) + ESP32-S3 (vision coprocessor), Lobot serial bus servos, A1M8 RPLIDAR, QMI8658 IMU, TTS/ASR I2C voice modules.

**Tech stack:** C++ (Arduino framework for ESP32), FreeRTOS, I2C, MadgwickAHRS, TFLite.

## Core Constraints

1. **全程自主运行** — 不能遥控。所有决策必须在板载代码中完成。
2. **限时** — Task 1 最多 5 分钟；Task 2+3 总共最多 5 分钟。
3. **碰撞扣 5 分/次** — 持续碰撞同一障碍物 → 任务直接失败。
4. **压线扣 5 分/次** — 一直压线 → 任务直接失败。
5. **识别/播报错误扣 10 分/个**。
6. **Code rules:** Don't delete `PillarNavigator/` or `ESP32S3Cam/` (classmate's Arduino IDE originals). Action group IDs must match hardware. No `delay()` blocking in main loop (use non-blocking timers).

## Architecture

### Dual-chip I2C communication

```
ESP32-S3 (slave 0x52)                  ESP32 (master)
  ├─ Ov2640 camera                     ├─ Serial2 → Lobot bus servos
  ├─ TFLite AI inference               ├─ GPIO4   → PWM head servo
  ├─ HSV color detection               ├─ UART    → A1M8 RPLIDAR
  └─ I2C registers:                    ├─ I2C     → QMI8658 IMU
       0x00 = red block data           ├─ I2C     → TTS module (0x40)
       0x01 = green block data         └─ I2C     → ASR module (0x34)
       0x02 = blue block data
       0x10 = AI label result
       0x20 = AI shape result
```

### Source layout

```
Esp32/src/              Esp32S3/src/
├── main.cpp            ├── main.cpp
├── globals.cpp         ├── camera_setting.*
├── base_config.h       ├── VisionAI.*
├── hw_esp32cam_ctl.*   ├── iic_data_send.*
├── voice/              ├── color_detection.*
│   ├── HWSensor.*      └── who_ai_utils.*
│   └── ShapeVoicePlayer.*
├── lidar/
│   └── LidarDriver.*       ← A1M8 360°扫描 + 形状分类
├── fall/
│   ├── TumbleGuard.*
│   └── FallDetector.*
├── tracking/
│   ├── HeadTracker.*       ← 头部舵机追踪色块
│   ├── ObjectFollower.*    ← 跟随色块状态机
│   ├── CirclePillar.*      ← 绕柱导航
│   └── RobotTracking.*     ← PID 视觉追踪
├── LobotServoCtl/
│   └── LobotServoController.cpp  ← 串口舵机驱动
├── PwmServo/
│   └── Servo.cpp                ← PWM 舵机驱动
└── IMU/
```

### Module status

| Module | Status | What it does |
|--------|--------|-------------|
| **LidarDriver** | ✅ Done | A1M8 360° scanning, obstacle detection, shape classification (sphere/cube/cylinder) |
| **IMU (QMI8658+Madgwick)** | ✅ Done | 6-axis orientation, roll/pitch output |
| **FallDetector / TumbleGuard** | ✅ Done | Two implementations of IMU-based fall detection + auto-stand action groups |
| **HeadTracker** | ✅ Done | Head servo tracks color target via I2C data from S3 |
| **ObjectFollower** | ✅ Done | State machine to follow color target (turn → forward → arrived) |
| **CirclePillar** | ✅ Done | Pillar navigation (left/right circling modes) |
| **RobotTracking** | ✅ Done | PID visual tracking (head servo + leg action groups) |
| **TTS/ASR + ShapeVoicePlayer** | ✅ Done | Voice synthesis + recognition; announces object name 3× |
| **ESP32Cam I2C** | ✅ Done | I2C master comms with S3 slave |
| **ESP32-S3 Vision** | ✅ Done | Camera + HSV detection + AI inference |
| **Servo drivers** | ✅ Done | PWM (LEDC) + Lobot serial bus servo protocols |

## Gap Analysis

### Critical (P0) — 比赛状态机缺失

`main.cpp` is currently a Lidar test program. Needs complete 3-task pipeline state machine.

| Priority | Module | What's needed |
|----------|--------|---------------|
| 🔴 P0 | **比赛状态机** | Top-level state machine sequencing Task1→2→3, with 5-min timeout per task |
| 🔴 P0 | **S型绕桩 (Task 1)** | Use Lidar + CirclePillar to navigate S-curve around 2 cylinders in obstacle zone |
| 🔴 P0 | **识别导航 (Task 2)** | After reaching end zone: read label → identify object → navigate to its mark area |
| 🔴 P0 | **语音播报 (Task 3)** | Call ShapeVoicePlayer 3× when at the correct mark area |
| 🟡 P1 | **通道导航** | Lidar centering algorithm to pass through 500×500mm channels |
| 🟡 P1 | **碰撞规避** | Real-time Lidar monitoring to stop/slow before collision |
| 🟡 P1 | **标签文字读取** | Read random text label shown by judge (S3 vision task) |
| 🟢 P2 | **扣分计数** | Runtime tracking of collision/line-cross counts |

### Recommended order

1. **Write competition state machine** in `main.cpp` — skeleton: Task1 → Task2 → Task3
2. **S-curve navigation** — Lidar locates 2 pillars → CirclePillar around each
3. **Channel navigation** — Lidar centering for 500×500 passages
4. **Label reading + object matching** — S3 reads text label → matches to color/shape
5. **Collision avoidance** — Real-time lidar guard
6. **Field testing + parameter tuning** — Action group IDs, circling thresholds

### Key function reference (for writing the state machine)

```cpp
// Lidar
lidar.begin(rx, tx, motorPin);
lidar.update();
lidar.isScanComplete();
lidar.findObjectInSector(startDeg, endDeg, maxDistMm);
lidar.classifyObjectAt(angleDeg);      // 1=sphere, 2=cube, 3=cylinder
lidar.isSectorClear(centerDeg, widthDeg, thresholdMm);

// Tracking
circlePillar_init();
circlePillar_update(color_reg, CIRCLE_LEFT|CIRCLE_RIGHT);
circlePillar_isDone();
objectFollower_init();
objectFollower_update(color_reg, deadband);
tracker_init();
tracker_set_color(TRACK_COLOR_RED|TRACK_COLOR_BLUE);
tracker_update();

// Fall detection
fallDetector_init();
fallDetector_update();
fallDetector_isRecovering();
tumble();

// Voice
ShapeVoicePlayer player;
player.playSphere(); player.playCube(); player.playCylinder();

// IMU
imu.begin();
imu.get_angle(&roll, &pitch);

// Servo action groups (match hardware!)
Controller.runActionGroup(groupId, times);
// 101=back-stand  102=front-stand  11=slide-left  12=slide-right
// 23=turn-left    24=turn-right    25=forward     3=small-left
// 4=small-right   0=stand         19=quick-stand
```
