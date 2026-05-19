#include "LidarDriver.h"

// ============================================================
// 构造函数
// ============================================================
LidarDriver::LidarDriver()
    : m_motorPin(-1)
    , m_lastPointMs(0)
    , m_scanReady(false)
    , m_connected(false)
    , m_lastAngle(-1)
    , m_pointCount(0)
{
    for (int i = 0; i < LIDAR_MAP_SIZE; i++) {
        m_map[i]     = 0;
        m_quality[i] = 0;
    }
}

LidarDriver::~LidarDriver() {
    end();
}

// ============================================================
// begin() —— 初始化 A1M8 激光雷达
//
// 注意：RPLidar 库的 begin() 内部会调 Serial2.begin(115200)
// 但不传引脚参数，导致串口回到默认引脚(16/17)。
// 必须在库 begin() 之后重新配置 Serial2 引脚。
// ============================================================
bool LidarDriver::begin(int rxPin, int txPin, int motorPin) {
    m_motorPin = motorPin;

    Serial.printf("[LIDAR] 初始化: RX=GPIO%d TX=GPIO%d MOTOR=GPIO%d\n",
                  rxPin, txPin, motorPin);

    pinMode(m_motorPin, OUTPUT);
    digitalWrite(m_motorPin, LOW);

    // ——————————————————————————————————————————————
    // 1. 初始化 Serial2 并启动电机
    // ——————————————————————————————————————————————
    Serial2.begin(115200, SERIAL_8N1, rxPin, txPin);
    delay(100);

    Serial.println("[LIDAR] 启动电机...");
    digitalWrite(m_motorPin, HIGH);
    delay(2000);  // 等电机稳定

    // 清空电机启动期间堆积的扫描数据
    while (Serial2.available()) Serial2.read();

    // ——————————————————————————————————————————————
    // 2. 绑定 RPLidar 库
    // ——————————————————————————————————————————————
    m_lidar.begin(Serial2);

    // ⚠ 库的 begin() 内部调了 Serial2.begin(115200) → 引脚被重设回默认！
    //    必须重新配置成我们的引脚
    Serial2.end();
    delay(10);
    Serial2.begin(115200, SERIAL_8N1, rxPin, txPin);
    delay(100);
    while (Serial2.available()) Serial2.read();  // 清干净

    // ——————————————————————————————————————————————
    // 3. 获取设备信息验证通信
    // ——————————————————————————————————————————————
    rplidar_response_device_info_t info;
    u_result result = m_lidar.getDeviceInfo(info, 1000);

    if (!IS_OK(result)) {
        Serial.printf("[LIDAR] getDeviceInfo 失败: 0x%X\n", result);
        Serial.println("[LIDAR] 检查 A1M8 供电(两个5V脚都需要)和接线");
        digitalWrite(m_motorPin, LOW);
        return false;
    }

    Serial.printf("[LIDAR] 检测到设备: 型号=0x%X 固件=v%d.%d\n",
                  info.model,
                  info.firmware_version >> 8,
                  info.firmware_version & 0xFF);

    // ——————————————————————————————————————————————
    // 4. 启动扫描
    // ——————————————————————————————————————————————
    delay(100);
    while (Serial2.available()) Serial2.read();

    result = m_lidar.startScan();
    Serial.printf("[LIDAR] startScan: 0x%X\n", result);

    if (!IS_OK(result)) {
        Serial.printf("[LIDAR] startScan 失败: 0x%X\n", result);
        digitalWrite(m_motorPin, LOW);
        return false;
    }

    // 等一会确认数据流入
    delay(500);
    int avail = Serial2.available();
    Serial.printf("[LIDAR] 扫描启动后缓冲区: %d 字节\n", avail);

    m_connected = true;
    m_lastPointMs = millis();
    Serial.println("[LIDAR] ✓ 雷达就绪！");
    return true;
}

// ============================================================
// end() —— 停止雷达
// ============================================================
void LidarDriver::end() {
    if (m_motorPin >= 0) {
        digitalWrite(m_motorPin, LOW);
    }
    m_lidar.stop();
    m_lidar.end();
    m_connected = false;
}

// ============================================================
// update() —— 处理一个扫描点
// ============================================================
bool LidarDriver::update() {
    if (!m_connected) return false;

    if (IS_OK(m_lidar.waitPoint())) {
        float dist   = m_lidar.getCurrentPoint().distance;
        float angle  = m_lidar.getCurrentPoint().angle;
        bool  start  = m_lidar.getCurrentPoint().startBit;
        uint8_t qual = m_lidar.getCurrentPoint().quality;

        int idx = ((int)(angle + 0.5f)) % LIDAR_MAP_SIZE;

        if (start && m_lastAngle > 350 && idx < 10) {
            m_scanReady = true;
        }
        m_lastAngle = idx;

        if (qual > 0 && dist > 0) {
            m_map[idx]     = dist;
            m_quality[idx] = qual;
        }

        m_lastPointMs = millis();
        m_pointCount++;
        return true;
    }

    // 看门狗
    if (m_pointCount == 0 && millis() - m_lastPointMs > 5000) {
        Serial.println("[LIDAR] 等待首帧数据...");
        m_lastPointMs = millis();
        return false;
    }
    if (m_pointCount > 0 && millis() - m_lastPointMs > 3000) {
        Serial.println("[LIDAR] 看门狗超时，尝试重连...");
        m_connected = false;
        digitalWrite(m_motorPin, LOW);
        delay(500);
        digitalWrite(m_motorPin, HIGH);
        delay(1000);
        m_lidar.startScan();
        m_lastPointMs = millis();
        m_connected = true;
    }

    return false;
}

// ============================================================
// 扫描状态
// ============================================================
bool LidarDriver::isScanComplete() { return m_scanReady; }
void LidarDriver::resetScanFlag()  { m_scanReady = false; }

// ============================================================
// 角度查询
// ============================================================
float LidarDriver::distanceAt(int deg) const {
    deg = ((deg % LIDAR_MAP_SIZE) + LIDAR_MAP_SIZE) % LIDAR_MAP_SIZE;
    return m_map[deg];
}

// ============================================================
// 在角度范围中找最远/最近的方向
// ============================================================
int LidarDriver::findFarthest(int startDeg, int endDeg, int stepDeg) const {
    float maxDist = 0;
    int bestAngle = -1;

    while (startDeg < 0) startDeg += LIDAR_MAP_SIZE;
    while (endDeg   < 0) endDeg   += LIDAR_MAP_SIZE;
    startDeg %= LIDAR_MAP_SIZE;
    endDeg   %= LIDAR_MAP_SIZE;

    auto check = [&](int a) {
        if (m_map[a] > maxDist) { maxDist = m_map[a]; bestAngle = a; }
    };

    if (startDeg <= endDeg) {
        for (int a = startDeg; a <= endDeg; a += stepDeg) check(a);
    } else {
        for (int a = startDeg; a < LIDAR_MAP_SIZE; a += stepDeg) check(a);
        for (int a = 0; a <= endDeg; a += stepDeg) check(a);
    }
    return bestAngle;
}

int LidarDriver::findNearest(int startDeg, int endDeg, int stepDeg) const {
    float minDist = 1e9;
    int bestAngle = -1;

    while (startDeg < 0) startDeg += LIDAR_MAP_SIZE;
    while (endDeg   < 0) endDeg   += LIDAR_MAP_SIZE;
    startDeg %= LIDAR_MAP_SIZE;
    endDeg   %= LIDAR_MAP_SIZE;

    auto check = [&](int a) {
        float d = m_map[a];
        if (d > 0 && d < minDist) { minDist = d; bestAngle = a; }
    };

    if (startDeg <= endDeg) {
        for (int a = startDeg; a <= endDeg; a += stepDeg) check(a);
    } else {
        for (int a = startDeg; a < LIDAR_MAP_SIZE; a += stepDeg) check(a);
        for (int a = 0; a <= endDeg; a += stepDeg) check(a);
    }
    return (minDist < 1e8) ? bestAngle : -1;
}

// ============================================================
// 扇形区域统计
// ============================================================
float LidarDriver::sectorMin(int centerDeg, int widthDeg) const {
    float d = 1e9;
    int half = widthDeg / 2;
    for (int off = -half; off <= half; off++) {
        float v = distanceAt(centerDeg + off);
        if (v > 0 && v < d) d = v;
    }
    return (d < 1e8) ? d : 0;
}

float LidarDriver::sectorMax(int centerDeg, int widthDeg) const {
    float d = 0;
    int half = widthDeg / 2;
    for (int off = -half; off <= half; off++) {
        float v = distanceAt(centerDeg + off);
        if (v > d) d = v;
    }
    return d;
}

float LidarDriver::sectorAvg(int centerDeg, int widthDeg) const {
    float sum = 0;
    int count = 0;
    int half = widthDeg / 2;
    for (int off = -half; off <= half; off++) {
        float v = distanceAt(centerDeg + off);
        if (v > 0) { sum += v; count++; }
    }
    return (count > 0) ? sum / count : 0;
}

bool LidarDriver::isSectorClear(int centerDeg, int widthDeg, float thresholdMM) const {
    int half = widthDeg / 2;
    for (int off = -half; off <= half; off++) {
        float v = distanceAt(centerDeg + off);
        if (v > 0 && v < thresholdMM) return false;
    }
    return true;
}

// ============================================================
// 调试输出
// ============================================================
void LidarDriver::printSummary() {
    int farthest  = findFarthest(0, 359);
    int nearest   = findNearest(0, 359);

    Serial.printf("[LIDAR] 点:%u  ", m_pointCount);
    if (farthest >= 0)
        Serial.printf("最远: %d° %.0fmm  ", farthest, m_map[farthest]);
    if (nearest >= 0)
        Serial.printf("最近: %d° %.0fmm", nearest, m_map[nearest]);
    Serial.println();

    Serial.printf("  前(0°):%.0f  右(90°):%.0f  后(180°):%.0f  左(270°):%.0f mm\n",
                  distanceAt(0), distanceAt(90), distanceAt(180), distanceAt(270));
}

// ============================================================
// findObjectInSector() —— 在扇形区域中找到最近的物体
// ============================================================
int LidarDriver::findObjectInSector(int startDeg, int endDeg, float maxDistMm) const {
    float minDist = maxDistMm;
    int bestAngle = -1;

    startDeg = ((startDeg % 360) + 360) % 360;
    endDeg   = ((endDeg   % 360) + 360) % 360;

    auto check = [&](int a) {
        float d = m_map[a];
        if (d > 20 && d < minDist) { minDist = d; bestAngle = a; }
    };

    if (startDeg <= endDeg) {
        for (int a = startDeg; a <= endDeg; a++) check(a);
    } else {
        for (int a = startDeg; a < 360; a++) check(a);
        for (int a = 0; a <= endDeg;    a++) check(a);
    }
    return bestAngle;
}

// ============================================================
// classifyObjectAt() —— 用激光雷达识别物体形状
//
// 原理：
//   立方体 → 平面反馈（距离标准差小）+ 边缘陡峭
//   圆柱体 → 圆弧面，角宽度约为 2·asin(150/(D+150))
//   球体   → 圆弧面（截面半径 < 150mm），角宽度明显小于圆柱体
//
// 注意：调用前确保 isScanComplete() 为 true（一圈完整扫描数据）
// ============================================================
int LidarDriver::classifyObjectAt(int angleDeg, ShapeInfo* info) {
    ShapeInfo local;
    if (!info) info = &local;
    memset(info, 0, sizeof(ShapeInfo));

    info->centerAngle = angleDeg;
    info->shapeType = 0;

    float centerDist = distanceAt(angleDeg);
    if (centerDist < 30) return 0;              // 无数据
    if (centerDist > 2000) return 0;            // 太远无法分类

    auto wrap = [](int d) { return ((d % 360) + 360) % 360; };

    // ——————————————————————————————————————————————
    // 1. 找到物体所在的连续角度簇
    // ——————————————————————————————————————————————
    // 从中心向两侧扩张，条件是：
    //   距离连续（与前一片的差值 < maxJump）且
    //   距离在合理范围（< 背景阈值）
    float bgThreshold = fmaxf(centerDist * 2.0f, 800.0f);  // 超过此值视为背景

    int leftDeg = angleDeg, rightDeg = angleDeg;

    // 向左扩张
    for (int i = 0; i < 60; i++) {
        int prev = wrap(leftDeg - 1);
        if (prev == rightDeg) break;             // 绕了一圈回来了
        float d = m_map[prev];
        if (d <= 0 || d > bgThreshold) break;    // 进入背景或无效
        float curr = m_map[leftDeg];
        if (curr > 0 && fabsf(d - curr) > 300) break;  // 距离跳变太大
        leftDeg = prev;
    }

    // 向右扩张
    for (int i = 0; i < 60; i++) {
        int next = wrap(rightDeg + 1);
        if (next == leftDeg) break;
        float d = m_map[next];
        if (d <= 0 || d > bgThreshold) break;
        float curr = m_map[rightDeg];
        if (curr > 0 && fabsf(d - curr) > 300) break;
        rightDeg = next;
    }

    int span;
    if (rightDeg >= leftDeg) {
        span = rightDeg - leftDeg + 1;
    } else {
        span = (360 - leftDeg) + rightDeg + 1;
    }
    info->angularSpan = span;
    if (span < 4) return 0;                     // 簇太小，无法分类

    // ——————————————————————————————————————————————
    // 2. 统计簇内距离值
    // ——————————————————————————————————————————————
    float sum = 0, sumSq = 0;
    float minD = 1e9f, maxD = 0;
    int count = 0;

    // 簇内的边缘梯度
    float leftEdgeGrad = 0, rightEdgeGrad = 0;
    float internalGradSum = 0;
    int internalGradCount = 0;

    int deg = leftDeg;
    for (int i = 0; i < span; i++) {
        float d = m_map[deg];
        if (d > 0) {
            sum += d;
            sumSq += d * d;
            if (d < minD) minD = d;
            if (d > maxD) maxD = d;
            count++;
        }

        // 每格的梯度（与相邻格的距离差）
        float grad = 0;
        int prev = wrap(deg - 1);
        int next = wrap(deg + 1);
        if (m_map[prev] > 0 && m_map[next] > 0) {
            grad = fabsf(m_map[next] - m_map[prev]) * 0.5f;
        }

        if (i == 0) {
            leftEdgeGrad = grad;                 // 左边缘梯度
        } else if (i == span - 1) {
            rightEdgeGrad = grad;                // 右边缘梯度
        } else if (i > 3 && i < span - 4) {
            internalGradSum += grad;             // 内部梯度（排除边缘附近）
            internalGradCount++;
        }

        deg = wrap(deg + 1);
    }

    info->minDist = minD;
    info->maxDist = maxD;

    if (count < 4) return 0;

    float mean = sum / count;
    float variance = sumSq / count - mean * mean;
    if (variance < 0) variance = 0;
    float stddev = sqrtf(variance);
    info->distStddev = stddev;

    // ——————————————————————————————————————————————
    // 3. 判断是否为立方体（平面 + 锐利边缘）
    // ——————————————————————————————————————————————
    float internalGrad = (internalGradCount > 0) ? (internalGradSum / internalGradCount) : 1.0f;
    bool sharpEdges = (leftEdgeGrad > internalGrad * 2.5f &&
                      rightEdgeGrad > internalGrad * 2.5f);
    info->sharpEdges = sharpEdges;

    // 立方体：平面（低标准差）+ 边缘陡峭
    if (sharpEdges && stddev < 25.0f) {
        info->shapeType = 2;    // 正方体
        return 2;
    }

    // ——————————————————————————————————————————————
    // 4. 区分球体与圆柱体（弧面物体）
    // ——————————————————————————————————————————————
    // 圆柱体预期角宽度：2·asin(R / (D+R))，R=150mm
    float expectedRad = 2.0f * asinf(150.0f / (minD + 150.0f));
    float expectedDeg = expectedRad * 180.0f / 3.14159265f;

    // 实际角宽度与预期值的比值
    float spanRatio = (float)span / expectedDeg;

    // 调试输出
    Serial.printf("[LIDAR_SHAPE] angle=%d  span=%d°  minD=%.0f  stddev=%.1f"
                  "  internalGrad=%.1f  leftG=%.1f  rightG=%.1f"
                  "  expCyl=%d°  ratio=%.2f\n",
                  angleDeg, span, minD, stddev,
                  internalGrad, leftEdgeGrad, rightEdgeGrad,
                  (int)expectedDeg, spanRatio);

    // 比值判断：
    //   spanRatio ≈ 1.0~1.3 → 立方体或被完整观测到的柱体 → 圆柱
    //   spanRatio ≈ 0.5~0.8 → 截面明显偏小 → 球体（激光雷达在球体下半部）
    if (spanRatio > 0.78f) {
        info->shapeType = 3;    // 圆柱体
    } else {
        info->shapeType = 1;    // 球体（截面比圆柱小）
    }

    return info->shapeType;
}

void LidarDriver::printPolarPlot() {
    Serial.println("[LIDAR] 极坐标图 (前方0° 顺时针):");
    for (int deg = 0; deg < 360; deg += 15) {
        float d = distanceAt(deg);
        int bars = (d > 0) ? constrain(map((int)d, 0, 4000, 1, 40), 1, 40) : 0;

        char buf[64];
        int n = snprintf(buf, sizeof(buf), "%3d°", deg);
        if (d > 0)
            n += snprintf(buf + n, sizeof(buf) - n, " %4.0f ", d);
        else
            n += snprintf(buf + n, sizeof(buf) - n, " ---- ");

        Serial.print(buf);
        for (int i = 0; i < bars; i++) Serial.print('#');
        Serial.println();
    }
}
