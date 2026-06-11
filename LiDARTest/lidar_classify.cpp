/*
 * lidar_classify.cpp — 物体形状识别
 *
 * 从 Esp32/src/lidar/LidarDriver.cpp 移植，适配 g_map[] 数组
 * 算法已由同学A 验证过原理，B 可能需要在实测中调整阈值
 */

#include "lidar_classify.h"
#include <math.h>

int classifyObject(int angleDeg, ShapeInfo* info) {
    ShapeInfo local;
    if (!info) info = &local;
    memset(info, 0, sizeof(ShapeInfo));

    info->centerAngle = angleDeg;

    float centerDist = g_map[(angleDeg % 360 + 360) % 360];
    if (centerDist < 30) return 0;
    if (centerDist > 2000) return 0;

    // ---- 辅助函数 ----
    auto wrap = [](int d) { return ((d % 360) + 360) % 360; };

    // ---- 1. 找到物体所在的连续角度簇 ----
    float bgThreshold = fmaxf(centerDist * 2.0f, 800.0f);

    int leftDeg = angleDeg, rightDeg = angleDeg;

    // 向左扩张
    for (int i = 0; i < 60; i++) {
        int prev = wrap(leftDeg - 1);
        if (prev == rightDeg) break;
        float d = g_map[prev];
        if (d <= 0 || d > bgThreshold) break;
        float curr = g_map[leftDeg];
        if (curr > 0 && fabsf(d - curr) > 300) break;
        leftDeg = prev;
    }

    // 向右扩张
    for (int i = 0; i < 60; i++) {
        int next = wrap(rightDeg + 1);
        if (next == leftDeg) break;
        float d = g_map[next];
        if (d <= 0 || d > bgThreshold) break;
        float curr = g_map[rightDeg];
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
    if (span < 4) return 0;

    // ---- 2. 统计簇内距离值 ----
    float sum = 0, sumSq = 0;
    float minD = 1e9f, maxD = 0;
    int count = 0;

    float leftEdgeGrad = 0, rightEdgeGrad = 0;
    float internalGradSum = 0;
    int internalGradCount = 0;

    int deg = leftDeg;
    for (int i = 0; i < span; i++) {
        float d = g_map[deg];
        if (d > 0) {
            sum += d;
            sumSq += d * d;
            if (d < minD) minD = d;
            if (d > maxD) maxD = d;
            count++;
        }

        float grad = 0;
        int prev = wrap(deg - 1);
        int next = wrap(deg + 1);
        if (g_map[prev] > 0 && g_map[next] > 0) {
            grad = fabsf(g_map[next] - g_map[prev]) * 0.5f;
        }

        if (i == 0) {
            leftEdgeGrad = grad;
        } else if (i == span - 1) {
            rightEdgeGrad = grad;
        } else if (i > 3 && i < span - 4) {
            internalGradSum += grad;
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

    // ---- 3. 分类指标 ----
    float internalGrad = (internalGradCount > 0) ? (internalGradSum / internalGradCount) : 999.0f;
    bool sharpEdges = (leftEdgeGrad > internalGrad * 2.5f &&
                      rightEdgeGrad > internalGrad * 2.5f);
    info->sharpEdges = sharpEdges;

    float expectedRad = 2.0f * asinf(150.0f / (minD + 150.0f));
    float expectedDeg = expectedRad * 180.0f / 3.14159265f;
    float spanRatio = (float)span / expectedDeg;

    float flatness = (maxD - minD) / mean;

    // ---- 4. 分类 ----
    // 立方体：平坦 + 梯度小
    if (stddev < 8.0f && flatness < 0.03f && internalGrad < 2.0f) {
        return 2;
    }

    // 球体：角宽明显小于 300mm 圆柱预期
    if (spanRatio <= 0.78f) {
        return 1;
    }

    return 3; // 圆柱体
}
