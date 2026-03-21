#include "auto_binary.h"
#include "zf_device_mt9v03x.h"

#define AUTO_ROI_Y_START            (20)
#define AUTO_ROI_Y_END              (100)
#define AUTO_ROI_X_START            (10)
#define AUTO_ROI_X_END              (178)

#define AUTO_THRESHOLD_MIN          (20)
#define AUTO_THRESHOLD_MAX          (230)

#define AUTO_EXPOSURE_MIN           (100)
#define AUTO_EXPOSURE_MAX           (800)
#define AUTO_EXPOSURE_STEP          (8)

#define AUTO_TARGET_MEAN_LOW        (85)
#define AUTO_TARGET_MEAN_HIGH       (115)

#define AUTO_EXPOSURE_UPDATE_PERIOD (3)

uint8  g_auto_threshold = 120;
uint16 g_auto_exposure  = MT9V03X_EXP_TIME_DEF;

static uint8 s_exposure_update_div = 0;

//---------------------------------------------------
// 计算 ROI 平均灰度
//---------------------------------------------------
static uint8 auto_get_mean_gray(const uint8 image[AUTO_IMAGE_H][AUTO_IMAGE_W])
{
    uint32 sum = 0;
    uint32 count = 0;
    uint16 x, y;

    for(y = AUTO_ROI_Y_START; y < AUTO_ROI_Y_END; y++)
    {
        for(x = AUTO_ROI_X_START; x < AUTO_ROI_X_END; x++)
        {
            sum += image[y][x];
            count++;
        }
    }

    if(count == 0)
    {
        return 0;
    }

    return (uint8)(sum / count);
}

//---------------------------------------------------
// Otsu 自动阈值
//---------------------------------------------------
static uint8 auto_calc_otsu_threshold(const uint8 image[AUTO_IMAGE_H][AUTO_IMAGE_W])
{
    uint32 hist[256] = {0};
    uint32 total = 0;
    uint32 sum = 0;
    uint32 sum_b = 0;
    uint32 w_b = 0;
    uint32 w_f = 0;

    float max_var = -1.0f;
    uint8 threshold = 120;

    uint16 x, y, i;

    for(y = AUTO_ROI_Y_START; y < AUTO_ROI_Y_END; y++)
    {
        for(x = AUTO_ROI_X_START; x < AUTO_ROI_X_END; x++)
        {
            hist[image[y][x]]++;
            total++;
        }
    }

    if(total == 0)
    {
        return threshold;
    }

    for(i = 0; i < 256; i++)
    {
        sum += i * hist[i];
    }

    for(i = 0; i < 256; i++)
    {
        w_b += hist[i];
        if(w_b == 0)
        {
            continue;
        }

        w_f = total - w_b;
        if(w_f == 0)
        {
            break;
        }

        sum_b += i * hist[i];

        {
            float m_b = (float)sum_b / (float)w_b;
            float m_f = (float)(sum - sum_b) / (float)w_f;
            float diff = m_b - m_f;
            float var_between = (float)w_b * (float)w_f * diff * diff;

            if(var_between > max_var)
            {
                max_var = var_between;
                threshold = (uint8)i;
            }
        }
    }

    if(threshold < AUTO_THRESHOLD_MIN)
    {
        threshold = AUTO_THRESHOLD_MIN;
    }
    if(threshold > AUTO_THRESHOLD_MAX)
    {
        threshold = AUTO_THRESHOLD_MAX;
    }

    return threshold;
}

//---------------------------------------------------
// 自动曝光
//---------------------------------------------------
static void auto_adjust_exposure(const uint8 image[AUTO_IMAGE_H][AUTO_IMAGE_W])
{
    uint8 mean_gray;

    s_exposure_update_div++;
    if(s_exposure_update_div < AUTO_EXPOSURE_UPDATE_PERIOD)
    {
        return;
    }
    s_exposure_update_div = 0;

    mean_gray = auto_get_mean_gray(image);

    if(mean_gray < AUTO_TARGET_MEAN_LOW)
    {
        if(g_auto_exposure + AUTO_EXPOSURE_STEP <= AUTO_EXPOSURE_MAX)
        {
            g_auto_exposure += AUTO_EXPOSURE_STEP;
            mt9v03x_set_exposure_time(g_auto_exposure);
        }
    }
    else if(mean_gray > AUTO_TARGET_MEAN_HIGH)
    {
        if(g_auto_exposure >= AUTO_EXPOSURE_MIN + AUTO_EXPOSURE_STEP)
        {
            g_auto_exposure -= AUTO_EXPOSURE_STEP;
            mt9v03x_set_exposure_time(g_auto_exposure);
        }
    }
}

//---------------------------------------------------
// 二值化
//---------------------------------------------------
static void auto_binary_image(const uint8 image[AUTO_IMAGE_H][AUTO_IMAGE_W],
                              uint8 binary[AUTO_IMAGE_H][AUTO_IMAGE_W],
                              uint8 threshold)
{
    uint16 x, y;

    for(y = 0; y < AUTO_IMAGE_H; y++)
    {
        for(x = 0; x < AUTO_IMAGE_W; x++)
        {
            if(image[y][x] > threshold)
            {
                binary[y][x] = 255;
            }
            else
            {
                binary[y][x] = 0;
            }
        }
    }
}

//---------------------------------------------------
// 初始化
//---------------------------------------------------
void auto_process_init(void)
{
    g_auto_threshold = 120;
    g_auto_exposure  = MT9V03X_EXP_TIME_DEF;
    s_exposure_update_div = 0;

    mt9v03x_set_exposure_time(g_auto_exposure);
}

//---------------------------------------------------
// 每帧处理：自动曝光 + 自动阈值 + 二值化
//---------------------------------------------------
void auto_process(const uint8 image[AUTO_IMAGE_H][AUTO_IMAGE_W],
                  uint8 binary[AUTO_IMAGE_H][AUTO_IMAGE_W])
{
    auto_adjust_exposure(image);
    g_auto_threshold = auto_calc_otsu_threshold(image);
    auto_binary_image(image, binary, g_auto_threshold);
}