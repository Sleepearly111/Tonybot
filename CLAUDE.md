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

## Current Status (2026-05-26)

**正在进行的任务：** 比赛状态机 + 3物体信标导航实现

### 场地布局

```
总宽 = 2100mm，总深 = 4400mm，所有 500 宽区域均居中

起点区(600×500) → 通道一(500×500) → 避障区(2100×1300) → 通道二(500×500)
  → 大终点区(600×500) → 空区(500×500) → 置物台(1800×400, 高300)

小起点: 200×200 居中于大起点区
小终点: 200×200 居中于大终点区
置物台上: 3物体(球φ200-300 / 正方体300³ / 圆柱φ300×300)，间距300mm
标记区: 3个150×150胶带框，距置物台200mm

识别区 = 通道二(500) + 大终点区(600) + 空区(500) + 置物台(400) = 2000mm深
```

### 传感器约束

- **LiDAR 平面 > 300mm**：全场只能看到置物台上的 3 个物体
- 柱子高 300mm → LiDAR 平面以下，扫不到（仅相机看颜色）
- 置物台高 300mm → LiDAR 扫不到台面
- 地上胶带/标记区(150×150) → 相机和 LiDAR 都看不到

### 导航策略

**核心思路：3 个物体从头到尾在 LiDAR 视野中，作为全程导航信标**

| 阶段 | 传感器 | 方法 |
|------|--------|------|
| 通道一穿越 | IMU + LiDAR | 航向保持 + 3物体星座居中 |
| 避障区绕柱 (Task 1) | 相机 | HeadTracker + ObjectFollower + CirclePillar |
| 找通道二入口 | LiDAR | 原地旋转扫描 3 物体方向 |
| 通道二穿越 | LiDAR | 3物体星座居中导航 |
| 物体识别 (Task 2) | LiDAR | classifyObjectAt() 多次投票 |
| 标记区导航+播报 (Task 3) | LiDAR + TTS | 以物体为参照接近到 200mm → ShapeVoicePlayer 3次 |

### 已知问题

| Issue | Severity | Status |
|-------|----------|--------|
| **TG1WDT_SYS_RESET** — `setup()` 中累计 `delay()` 超过 5 秒触发硬件看门狗复位 | 🔴 阻塞 | 修复中 |
| CirclePillar CIRCLING→DONE 无自动退出条件 | 🟡 已知 | 待修复 |
| action group 不一致: ObjectFollower 用 25 forward, RobotTracking 用 21 | 🟡 已知 | 待场地确认 |
| 形状分类阈值 — 1° 分辨率下 >400mm 球体/圆柱体难区分 | 🟡 已知 | 待场地实测 |

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

### Critical (P0)

`main.cpp` is currently a LiDAR test program. Needs complete 3-task pipeline state machine with 3-object beacon navigation.

| Priority | Module | What's needed |
|----------|--------|---------------|
| 🔴 P0 | **比赛状态机** | Top-level state machine: Task1(相机绕柱) → Task2(LiDAR识别) → Task3(标记区+播报) |
| 🔴 P0 | **IMU yaw 暴露** | Add `get_yaw()` to Hiwonder.hpp, used for short heading holds |
| 🔴 P0 | **3物体星座导航** | LiDAR detects 3 objects → compute center angle → steer to keep objects centered |
| 🔴 P0 | **通道穿越** | IMU heading hold + 3-object constellation centering through 500×500 channels |
| 🟡 P1 | **碰撞规避** | Real-time LiDAR front monitoring to stop before collision |
| 🟡 P1 | **WDT-safe delay** | Replace all `delay()` in setup() with watchdog-feeding variant |
| 🟢 P2 | **扣分计数** | Runtime tracking of collision/line-cross counts |

### Recommended order

1. **Infrastructure** — arena.h constants, wdt_util.h (safeDelay), expose IMU yaw
2. **3-object beacon navigation** — `nav_alignToObjects()`, `nav_findObjectsDirection()`, `nav_approachObject()`
3. **State machine skeleton** — in `main.cpp`: Task1 → Task2 → Task3 with timeout guards
4. **Channel crossing** — IMU heading hold + beacon centering
5. **Pillar circling fix** — Add DONE exit condition to CirclePillar
6. **Field testing + parameter tuning** — Action group IDs, circling thresholds, classifyObjectAt thresholds

### Arena constants reference

```cpp
// arena.h — all in mm
#define ARENA_TOTAL_DEPTH  4400  // 600+500+1300+500+600+500+400
#define ARENA_TOTAL_WIDTH  2100
#define NARROW_ZONE_WIDTH  500
#define OBSTACLE_ZONE_DEPTH 1300
#define RECOG_ZONE_DEPTH   2000  // channel2+end_zone+gnd_gap+platform
#define CHANNEL_LENGTH     500
#define PLATFORM_WIDTH     1800
#define PLATFORM_DEPTH     400
#define PLATFORM_HEIGHT    300
#define OBJECT_SPACING     300   // center-to-center between objects on platform
#define MARKER_OFFSET      200   // distance from platform to marker boxes
#define MARKER_SIZE        150   // marker box side length
#define START_BOX_SIZE     200   // small start/end box
#define LARGE_ZONE_DEPTH   600   // large start/end zone depth
#define LIDAR_SCAN_HEIGHT  300   // LiDAR plane is above this → only sees objects on platform
```

### Key function reference (for writing the state machine)

```cpp
// LiDAR
lidar.begin(rx, tx, motorPin);
lidar.update();
lidar.isScanComplete();
lidar.findObjectInSector(startDeg, endDeg, maxDistMm);
lidar.classifyObjectAt(angleDeg);      // 1=sphere, 2=cube, 3=cylinder
lidar.sectorMin(centerDeg, widthDeg);  // min distance in sector

// 3-object beacon navigation (new)
nav_findObjectsDirection();      // rotate in place, scan for 3-object cluster
nav_alignToObjects();             // compute center offset, return correction
nav_approachObject(n, targetDist);// approach nth object to target distance
nav_crossChannel();               // IMU heading hold + beacon centering through 500mm

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
imu.get_yaw();  // new — for short heading holds (<5s)

// WDT-safe delay
safeDelay(ms);  // feeds TG1WDT during delay, use instead of delay()

// Servo action groups (match hardware!)
Controller.runActionGroup(groupId, times);
// 101=back-stand  102=front-stand  11=slide-left  12=slide-right
// 23=turn-left    24=turn-right    25=forward     3=small-left
// 4=small-right   0=stand         19=quick-stand
```
