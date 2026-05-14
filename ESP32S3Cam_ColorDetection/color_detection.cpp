#include "color_detection.hpp"
#include "esp_log.h"
#include "esp_camera.h"
#include "dl_image.hpp"
#include "fb_gfx.h"
#include "color_detector.hpp"
#include "who_ai_utils.hpp"

using namespace std;
using namespace dl;

static const char *TAG = "color_detection";

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static QueueHandle_t xQueueResult = NULL;

static bool gReturnFB = true;

// 全局颜色模式：0=红色，1=蓝色（由主控板通过 I2C 命令码切换）
uint8_t g_color_mode = 0;
static uint8_t g_last_color_mode = 0xFF; // 用于检测模式切换

color_data_t color_data[5];

// ColorDetector 对象指针，用于模式切换时重新构造
static ColorDetector *g_detector = NULL;

/* 颜色阈值 用户可在此处调整 */
vector<color_info_t> std_color_info = {
    {{151, 15, 70, 255, 90, 255}, 64, "red"},
    {{23, 34, 70, 255, 90, 255}, 64, "yellow"},
    {{45, 75, 70, 255, 90, 255}, 64, "green"},
    {{97, 117, 70, 255, 90, 255}, 64, "blue"},
    {{130, 155, 70, 255, 90, 255}, 64, "purple"}
};

// 颜色在 std_color_info 中的索引
#define COLOR_IDX_RED   0
#define COLOR_IDX_BLUE  3

/* 获取颜色检测的结果，提取最大色块数据 */
static void get_color_detection_result(uint16_t *image_ptr, int image_height, int image_width,
                                       vector<color_detect_result_t> &results, int color_data_index)
{
    // 空结果判断
    if (results.size() == 0) {
        color_data[color_data_index].center_x = 0;
        color_data[color_data_index].center_y = 0;
        color_data[color_data_index].width = 0;
        color_data[color_data_index].length = 0;
        return;
    }

    // 使用局部变量寻找最大色块（修复：不再使用全局变量 g_max_color_area）
    int max_area = 0;
    int max_idx = 0;
    for (int i = 0; i < results.size(); ++i) {
        if (results[i].area > max_area) {
            max_area = results[i].area;
            max_idx = i;
        }
    }

    // 在循环外部赋值
    color_data[color_data_index].center_x = (uint8_t)results[max_idx].center[0];
    color_data[color_data_index].center_y = (uint8_t)results[max_idx].center[1];
    color_data[color_data_index].width  = (uint8_t)(results[max_idx].box[2] - results[max_idx].box[0]);
    color_data[color_data_index].length = (uint8_t)(results[max_idx].box[3] - results[max_idx].box[1]);
}

static void task_process_handler(void *arg)
{
    camera_fb_t *frame = NULL;

    // 初始构造检测器
    g_detector = new ColorDetector();

    while (true) {
        // 检测颜色模式是否切换
        if (g_color_mode != g_last_color_mode) {
            g_last_color_mode = g_color_mode;

            // 重新构造 ColorDetector，只注册当前模式的单个颜色
            delete g_detector;
            g_detector = new ColorDetector();

            if (g_color_mode == 0) {
                // 红色模式：仅注册红色
                g_detector->register_color(
                    std_color_info[COLOR_IDX_RED].color_thresh,
                    std_color_info[COLOR_IDX_RED].area_thresh,
                    std_color_info[COLOR_IDX_RED].name);
            } else {
                // 蓝色模式：仅注册蓝色
                g_detector->register_color(
                    std_color_info[COLOR_IDX_BLUE].color_thresh,
                    std_color_info[COLOR_IDX_BLUE].area_thresh,
                    std_color_info[COLOR_IDX_BLUE].name);
            }
        }

        if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY)) {
            std::vector<std::vector<color_detect_result_t>> &results =
                g_detector->detect((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3});

            // 清零所有颜色数据
            for (int i = 0; i < COLOR_NUM; ++i) {
                color_data[i].center_x = 0;
                color_data[i].center_y = 0;
                color_data[i].width = 0;
                color_data[i].length = 0;
            }

            // 根据当前模式，仅提取对应颜色的数据
            if (g_color_mode == 0) {
                // 红色模式 → 写入 color_data[0]
                if (results.size() > 0) {
                    get_color_detection_result((uint16_t *)frame->buf, (int)frame->height,
                                               (int)frame->width, results[0], 0);
                }
            } else {
                // 蓝色模式 → 写入 color_data[3]
                if (results.size() > 0) {
                    get_color_detection_result((uint16_t *)frame->buf, (int)frame->height,
                                               (int)frame->width, results[0], 3);
                }
            }
        }

        if (xQueueFrameO) {
            xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
        } else if (gReturnFB) {
            esp_camera_fb_return(frame);
        } else {
            free(frame);
        }

        if (xQueueResult) {
            xQueueSend(xQueueResult, &color_data, portMAX_DELAY);
        }
    }
}

static void task_event_handler(void *arg)
{
    while (true) {
    }
}

void register_color_detection(const QueueHandle_t frame_i,
                                   const QueueHandle_t event,
                                   const QueueHandle_t result,
                                   const QueueHandle_t frame_o,
                                   const bool camera_fb_return)
{
    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    xQueueEvent = event;
    xQueueResult = result;
    gReturnFB = camera_fb_return;

    xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);
    // xTaskCreatePinnedToCore(task_event_handler, TAG, 4 * 1024, NULL, 5, NULL, 0);
}
