// search_line.c
#include "zf_common_headfile.h"
#include "zf_device_ips114.h"
#include "zf_device_mt9v03x.h"
#include "search_line.h"  // 引用 search_line.h

#define IMAGE_W 188
#define IMAGE_H 120
#define ROI_START 60
#define ROI_END 110

// 存储搜索得到的左右边界和中心线
int left_line[IMAGE_H];
int right_line[IMAGE_H];
int center_line[IMAGE_H];

int track_width = 60;  // 赛道宽度，可以根据实际情况调整

uint8 binary_image[IMAGE_H][IMAGE_W];  // 存储二值化后的图像

// 当前的偏差和Yaw值
static int current_offset = 0;  // 偏差值
static float current_yaw = 0.0f;  // Yaw值

/**
 * @brief 搜索赛道的左右边界和中心线，计算偏差并转换为Yaw角度
 * 
 * @return None
 * 
 * 该函数遍历ROI区域（即感兴趣区域）内的图像，搜索赛道的左右边界，计算中心线的位置，
 * 通过中心线与图像中心的偏差（offset）来计算Yaw角度。
 * 偏差和Yaw值存储在全局变量 `current_offset` 和 `current_yaw` 中。
 */
void search_line(void)
{
    for (int y = ROI_START; y < ROI_END; y++)
    {
        int left = -1;
        int right = -1;

        // 搜索左边界
        for (int x = 0; x < IMAGE_W; x++)
        {
            if (binary_image[y][x] == 255)
            {
                left = x;
                break;
            }
        }

        // 搜索右边界
        for (int x = IMAGE_W - 1; x >= 0; x--)
        {
            if (binary_image[y][x] == 255)
            {
                right = x;
                break;
            }
        }

        // 如果没有找到边界，则使用上一次的边界
        if (left < 0 || right < 0)
        {
            left = left_line[y - 1];
            right = right_line[y - 1];
        }

        // 存储找到的边界
        left_line[y] = left;
        right_line[y] = right;

        // 计算中心线
        center_line[y] = (left + right) / 2;
    }

    // 计算偏差，使用中心线的第90行（或根据需要调整）
    current_offset = center_line[90] - IMAGE_W / 2;

    // 将偏差转换为Yaw角度
    current_yaw = offset_to_yaw(current_offset, track_width);
}

/**
 * @brief 获取实时计算得到的Yaw角度
 * 
 * @return float 当前计算得到的Yaw角度
 * 
 * 该函数返回实时计算得到的Yaw角度，可以在主程序中直接调用。
 */
float get_yaw(void)
{
    return current_yaw;
}