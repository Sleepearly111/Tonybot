# Part2_EndZone — 终点区导航 + 文字驱动分类

CRAIC 2026 比赛第二阶段：从绕柱完成后的位置出发，LiDAR 导航到终点区，等待裁判文字指令，完成 3 轮物体分类。

---

## 比赛流程

```
绕柱完成(Part1) → 立正 → 烧录Part2 → 上电
                                      ↓
                            STAGE_NAV: LiDAR 导航到终点区
                            (以中间物体为信标, 走到 80cm 处)
                                      ↓
                            STAGE_ARRIVED: 等待裁判文字指令
                            (S3 切换 AI 模式, 连续3次确认同一文字)
                                      ↓
                            STAGE_CLASSIFY: 3 轮文字驱动分类
                            (等文字 → 分类 → 查表 → 侧滑 → 逼近 → 播报 → 偏头)
                                      ↓
                            STAGE_DONE: 立正, 完成
```

## 硬件连接

| 外设 | 引脚 | 说明 |
|------|------|------|
| LiDAR 串口 | RX=32, TX=33 (Serial1) | RPLidar A1, 115200bps |
| LiDAR 电机 | IO13 | HIGH=转, LOW=停 |
| 舵机控制板 | TX=17, RX=16 (Serial2) | 9600bps |
| 头部舵机 | IO4 | 0°~180° PWM |
| I2C SDA/SCL | IO22/IO23 | S3摄像头(0x52) + 语音模块(0x34) + 超声波(0x77) |
| 蜂鸣器 | IO21 | 跌倒报警 |
| 雷达串口 | Serial1 | 115200 |

## 目录结构

```
Part2_EndZone/
├── Part2_EndZone.ino          ← 主程序 (Arduino IDE 打开此文件)
├── lidar_common.cpp/h         ← LiDAR 初始化 + g_map[360] 全局数据 + NavCmd 结构体
├── lidar_navigate.cpp/h       ← 物体检测(lidar_findObjects) + 导航到终点区(lidar_nav_toPlatform)
├── lidar_endzone.cpp/h        ← 终点区判定(lidar_isAtEndZone)
├── lidar_classify.cpp/h       ← 物体形状识别(classifyObject + classifyThree)
├── lidar_action.cpp/h         ← 舵机动作执行(action_execute/action_slide/action_wait)
├── NavigateToObject.cpp/h     ← 3轮文字驱动分类状态机 (核心)
├── RPLidar.cpp/h              ← RPLidar A1 驱动
├── inc/                       ← 雷达协议头文件
├── pack/                      ← 第三方库
└── src/                       ← 驱动库 (IMU/蜂鸣器/温湿度/舵机)
```

---

## 核心状态机

### 顶层 4 阶段

| 阶段 | 枚举值 | 雷达电机 | S3 模式 | 功能 |
|------|--------|----------|---------|------|
| NAV | `STAGE_NAV` | ON (IO13=HIGH) | 任意 | LiDAR 导航到终点区 |
| ARRIVED | `STAGE_ARRIVED` | OFF (IO13=LOW) | AI 文字模式 | 等待裁判文字指令 |
| CLASSIFY | `STAGE_CLASSIFY` | ON→OFF→ON | AI 文字模式 | 3 轮文字驱动分类 |
| DONE | `STAGE_DONE` | OFF | — | 完成 |

### STAGE_NAV — LiDAR 导航

```
lidar_update() → 填充 g_map[360]
      ↓
g_scanReady == true → 开始一帧处理
      ↓
lidar_isAtEndZone()? ──YES──→ 关雷达, 立正, S3切AI模式, → STAGE_ARRIVED
      ↓ NO
lidar_nav_toPlatform() → NavCmd
      ↓
action_execute(cmd) → 舵机动作
```

**导航子阶段** (`lidar_navigate.cpp`):

| 子阶段 | 枚举值 | 行为 |
|--------|--------|------|
| 稳定 | 隐含在 NAV_FACE_OBJ | 等待中间物体角度连续 3 帧变化 < 8° |
| 转正 | `NAV_FACE_OBJ` | 转弯对准中间物体 (误差 ≤ 10°) |
| 逼近 | `NAV_APPROACH` | 直走靠近, 同时微调方向 |
| 到达 | `NAV_ARRIVED` | 返回 arrived=true |

### STAGE_ARRIVED — 等待文字 (比赛成败关键)

```
S3 切换到 AI 文字识别模式
      ↓
循环读取 readTextFromESP32() (I2C 0x52, 寄存器 0x10)
      ↓
读到 0x00 → sawZero=true (S3 已刷新, 可接收新数据)
      ↓
读到 0x11/0x12/0x13 → 与上次相同 → confirmCnt++
      ↓
confirmCnt >= 3 → 确认! 语音播报, 开雷达, → STAGE_CLASSIFY
```

**必须等到文字识别才能继续**，不能跳过。

### STAGE_CLASSIFY — 3 轮分类 (`NavigateToObject.cpp`)

每轮流程：

```
WAIT_TEXT → HEAD_BACK(回正90°) → CLASSIFY(5s等待+10帧确认) → SUMMARY(播报)
    → FIND_TARGET(查表) → SLIDE(侧滑+跌落计数) → APPROACH(4步逼近)
    → VOICE(播报×3) → TURN_HEAD(偏头0°) → WAIT_TEXT...
```

第 1 轮：头在 90° 等文字，识别后直接进入 CLASSIFY
第 2/3 轮：头偏开 0° 等文字，识别后 HEAD_BACK 回正再分类

**NTO 状态枚举** (`NavigateToObject.h`):

```cpp
enum NTO_Phase {
    NTO_GOTO_FIRST,    // 保留
    NTO_PARALLEL,      // 保留
    NTO_WAIT_TEXT,     // 等文字 (S3 AI模式, 雷达关闭)
    NTO_HEAD_BACK,     // 头回正到90°
    NTO_CLASSIFY,      // 分类: 5s等待收纸 + 连续10帧一致确认
    NTO_SUMMARY,       // 从左到右播报三个物体
    NTO_FIND_TARGET,   // 查表: 文字形状 → 目标位置
    NTO_SLIDE,         // 侧滑: 数跌落越过物体
    NTO_APPROACH,      // 直走逼近物体
    NTO_VOICE,         // 播报当前物体3次
    NTO_TURN_HEAD,     // 偏头到0°等下一文字
    NTO_DONE           // 3轮完成
};
```

---

## 全部可调参数

### Part2_EndZone.ino 顶层

| 参数 | 值 | 位置 | 说明 |
|------|-----|------|------|
| `ESP32S3_I2C_ADDR` | 0x52 | .ino:55 | S3 I2C 地址 |
| `ESP32S3_REG_LABEL` | 0x10 | .ino:56 | S3 文字结果寄存器 |
| `ESP32S3_REG_MODE` | 0x30 | .ino:57 | S3 模式切换寄存器 |
| `ESP32S3_MODE_AI` | 0x02 | .ino:58 | AI 文字识别模式命令 |
| 文字确认次数 | 3 | .ino:236 | STAGE_ARRIVED 连续确认次数 |
| 雷达启动等待 | 1500ms | .ino:244 | 开雷达后等待稳定 |
| loop delay | 30ms | .ino:314 | 主循环间隔 |

### lidar_common.h/cpp — LiDAR 驱动

| 参数 | 值 | 说明 |
|------|-----|------|
| `PIN_LIDAR_RX` | 32 | 雷达串口 RX |
| `PIN_LIDAR_TX` | 33 | 雷达串口 TX |
| `PIN_LIDAR_MOTOR` | 13 | 雷达电机控制 |
| `g_map[360]` | float 数组 | 360° 距离数据 (mm), 0=无效 |
| `g_qual[360]` | uint8_t 数组 | 信号质量 |
| `g_scanReady` | bool | 一圈扫描完成标志 |
| 雷达波特率 | 115200 | Serial1 |
| 电机启动等待 | 2000ms | lidar_init() |

### lidar_navigate.cpp — 物体检测 + 导航

**物体检测 (lidar_findObjects):**

| 参数 | 值 | 说明 |
|------|-----|------|
| `SEARCH_START` | 10 | 搜索起始角度 |
| `SEARCH_END` | 170 | 搜索结束角度 (前方 ±80°) |
| `OBJ_MIN_DIST` | 100mm | 有效点最小距离 |
| `OBJ_MAX_DIST` | 3000mm | 有效点最大距离 |
| `GAP_DEG` | 8° | 聚类最大角度间距 |
| `DIST_TOL` | 300mm | 聚类距离容忍 |
| `MIN_SPAN` | 3° | 物体最小角度跨度 |
| `MAX_SPAN` | 50° | 物体最大角度跨度 |

**导航 (lidar_nav_toPlatform):**

| 参数 | 值 | 说明 |
|------|-----|------|
| `ROBOT_FRONT` | 90 | 机器人正前方对应角度 |
| `FACE_OK` | 10° | 对准容忍误差 |
| `TARGET_MM` | 800mm | 目标距离 (距中间物体) |
| `ARRIVE_TOL` | 150mm | 到达容忍 |
| `TURN_ANGLE` | 15° | 每次转弯角度 |
| `MAX_FWD` | 300mm | 单次最大前进距离 |
| `STABLE_NEED` | 3 | 稳定需要连续帧数 |
| `STABLE_TOL` | 8° | 稳定容忍角度变化 |

### lidar_endzone.cpp — 终点区判定

| 参数 | 值 | 说明 |
|------|-----|------|
| `TARGET_DIST_MM` | 800mm | 目标距离 (到中间物体) |
| `TOLERANCE_MM` | 200mm | ±200mm 容忍范围 |
| 判定条件 | 中间物体距离在 [600, 1000] mm | 即认为到达终点区 |

### NavigateToObject.cpp — 3轮分类状态机

| 参数 | 值 | 说明 |
|------|-----|------|
| `ROBOT_FRONT` | 90 | 机器人正前方 |
| `VOICE_DELAY` | 2000ms | 播报持续 |
| `DROP_MM` | 300mm | 跌落检测阈值 |
| `APPROACH_STEPS` | 4 | 逼近步数 (第1轮) |
| 分类等待 | 5000ms | 等待收纸 |
| 分类确认帧数 | 10 | 连续10帧一致确认 |
| 侧滑暂停 | 3000ms | 跨2物体时中间暂停 |

**侧滑跌落检测逻辑:**
- 监测 90° 方向距离 (g_map[90])
- 记录峰值距离 (g_dropMaxD)
- 当前距离比峰值下降 > DROP_MM(300mm) → 跌落1次
- 跌落次数 = 需要跨越的物体数 → 到达目标

**侧滑预转弯:**
- 左滑前: 小左转 10° (`smallTurn=true`)
- 右滑前: 小右转 10°

### lidar_classify.cpp — 物体形状识别

**单物体分类 (classifyObject):**

| 参数 | 值 | 说明 |
|------|-----|------|
| 有效距离范围 | 30~2000mm | 超出返回未知 |
| 背景阈值 | max(centerDist×2.0, 800mm) | 超过视为背景 |
| 扩张距离差 | 300mm | 相邻点超过此差停止扩张 |
| 最小跨度 | 4° | 小于此跨度返回未知 |
| 圆柱预期直径 | 150mm | 用于计算预期角宽 |
| 正方体 std 阈值 | < 8.0 | 距离标准差 |
| 正方体 flatness 阈值 | < 0.03 | 平坦度 |
| 正方体 internalGrad 阈值 | < 2.0 | 内部梯度 |
| 球体 spanRatio 阈值 | ≤ 0.78 | 角宽比 < 圆柱预期的78% |

**三物体对比分类 (classifyThree):**
- 正方体: std 偏离平均值最远 + 角宽最大
- 剩下两个: 角宽大 = 圆柱体, 角宽小 = 球体
- 纯相对比较, 不依赖绝对阈值

### lidar_action.cpp — 舵机动作

| 参数 | 值 | 说明 |
|------|-----|------|
| `ACT_STAND` | 0 | 直立动作组 |
| `ACT_TURN_LEFT_S` | 34 | 小幅左转 |
| `ACT_TURN_RIGHT_S` | 35 | 小幅右转 |
| `ACT_TURN_LEFT` | 23 | 左转 |
| `ACT_TURN_RIGHT` | 24 | 右转 |
| `ACT_FORWARD` | 25 | 前进 |
| `ACT_SLIDE_LEFT` | 11 | 左侧滑 |
| `ACT_SLIDE_RIGHT` | 12 | 右侧滑 |
| `DEG_PER_ACT` | 15° | 每个动作对应转角 |
| `MM_PER_ACT` | 1mm | forwardMm 对应执行次数 |
| `MAX_TURN_TIMES` | 4 | 单次最大转弯次数 |
| `MAX_FWD_TIMES` | 4 | 单次最大前进次数 |
| `TURN_WAIT_MS` | 800ms | 转弯后等待 |
| `FWD_WAIT_MS` | 800ms | 前进后等待 |
| `SETTLE_MS` | 1500ms | 动作后稳定等待 |

### HeadTracker 相关 (共用)

| 参数 | 值 | 说明 |
|------|-----|------|
| 头部舵机角度范围 | 0°~180° | IO4 PWM |
| 舵机默认角度 | 90° | 正前方 |
| 偏头角度(等文字) | 0° | TURN_HEAD 状态 |
| 偏头持续时间 | 1500ms | 舵机转到位的等待 |
| I2C 读取 S3 | 4 bytes/frame | x, ?, width, ? |

---

## S3 通信协议

### I2C 地址: 0x52

**切换模式:**
- 写寄存器 0x30 = 0x02 → AI 文字识别模式

**读取文字结果:**
- 读寄存器 0x10 → 1 byte
- 0x00 = 无识别 / 空闲
- 0x11 = 球体
- 0x12 = 正方体
- 0x13 = 圆柱体

**文字确认逻辑:**
1. 必须先读到 0x00 (sawZero=true), 清空旧缓存
2. 读到有效形状 (0x11/0x12/0x13) 后, 连续 3 次相同才确认
3. 确认后语音播报, 进入分类流程

---

## 调试建议

### 串口输出格式

```
[12345] [导航] d=120/80/150cm | 90°=95cm    ← STAGE_NAV
[12345] [等文字] 等待指令...                  ← STAGE_ARRIVED
[12345] [分类] 球/正方体/圆柱 确认!           ← STAGE_CLASSIFY
  [侧滑] →位置2 左滑 90°=90cm 峰值=95cm 跌落1/2
  [逼近] 第1/4步 90°=30cm
```

### 常见问题

| 问题 | 可能原因 | 查看 |
|------|----------|------|
| 雷达无数据 | 电机未转 / 串口接反 | Serial1 是否 115200 |
| 找不到3个物体 | 距离太远/物体不在前方±80° | g_map[] 数据 |
| 导航无限循环 | 物体检测不稳定 | 稳定计数 STABLE_NEED |
| 文字不识别 | S3 未切换到 AI 模式 | 寄存器 0x30 写 0x02 |
| 侧滑过早停止 | DROP_MM 阈值太小/雷达抖动 | 查看跌落日志 |

---

## Claude Code 使用说明

此 README 记录了所有可调参数及其位置。调试时:

1. 先在 `Part2_EndZone.ino` 中找到对应阶段的 `case STAGE_XXX:`
2. 根据参数表定位到具体文件的行号附近
3. 修改参数后, 同步更新此 README 中的参数表

**文件依赖关系:**
```
Part2_EndZone.ino
  ├── lidar_common.h        → g_map[360], NavCmd, lidar_init/update
  ├── lidar_endzone.h       → lidar_isAtEndZone()
  ├── lidar_navigate.h      → lidar_nav_toPlatform(), lidar_findObjects()
  ├── lidar_action.h        → action_execute(), action_slide(), action_wait()
  ├── NavigateToObject.h    → nto_update(), nto_textReceived(), NTO_Phase
  └── lidar_classify.h      → classifyObject(), classifyThree()
```
