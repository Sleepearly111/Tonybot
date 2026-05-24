#include "PillarRoute.hpp"

// 四种流程的阶段数据 [route][phase] = { color, direction }
// 顺序：phase0 第一根柱子, phase1 第二根柱子
static const uint8_t ROUTE_TABLE[4][2][2] = {
    // ROUTE_BLUE_RIGHT：先蓝柱+右绕 → 后红柱+左绕
    [0] = { { TRACK_COLOR_BLUE, ROUTE_RIGHT },
            { TRACK_COLOR_RED,  ROUTE_LEFT  } },

    // ROUTE_BLUE_LEFT：先蓝柱+左绕 → 后红柱+右绕
    [1] = { { TRACK_COLOR_BLUE, ROUTE_LEFT  },
            { TRACK_COLOR_RED,  ROUTE_RIGHT } },

    // ROUTE_RED_RIGHT：先红柱+右绕 → 后蓝柱+左绕
    [2] = { { TRACK_COLOR_RED,  ROUTE_RIGHT },
            { TRACK_COLOR_BLUE, ROUTE_LEFT  } },

    // ROUTE_RED_LEFT：先红柱+左绕 → 后蓝柱+右绕
    [3] = { { TRACK_COLOR_RED,  ROUTE_LEFT  },
            { TRACK_COLOR_BLUE, ROUTE_RIGHT } },
};

static uint8_t route_index = 0;  // 当前流程编号
static uint8_t phase = 0;        // 当前阶段 0 或 1
static int    phase_rounds[2] = {6, 6};  // 两阶段各需前进轮数

void pillarRoute_start(PillarRoute route) {
    route_index = (uint8_t)route;
    phase = 0;
}

uint8_t pillarRoute_color() {
    return ROUTE_TABLE[route_index][phase][0];
}

uint8_t pillarRoute_direction() {
    return ROUTE_TABLE[route_index][phase][1];
}

int pillarRoute_rounds() {
    return phase_rounds[phase];
}

void pillarRoute_setRounds(int first, int second) {
    phase_rounds[0] = first;
    phase_rounds[1] = second;
}

bool pillarRoute_next() {
    phase++;
    return (phase >= 2);
}
