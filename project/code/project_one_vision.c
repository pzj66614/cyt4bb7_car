#include "zf_common_headfile.h"
#include "auto_binary.h"
#include "trans.h"
#include "project_one_vision.h"

/*********************************************************************************************************************
 * 文件名称：project_one_vision.c
 * 文件功能：科目一“绕桩区”摄像头导航模块
 *
 * 核心设计思想：
 * 1. 不直接找“锥桶中心”去撞间隙，而是找“安全通道”
 * 2. 双桶时：
 *      用 左桶右边界 + 安全距离
 *      与 右桶左边界 - 安全距离
 *      之间的中点作为目标点
 * 3. 单桶时：
 *      不取桶中心，而是向远离锥桶的一侧绕行
 * 4. 连续看不到锥桶时：
 *      不盲目前进，而是进入“摆扫搜索”状态
 *
 * 与控制层的对接方式：
 *      Target_Yaw_Angle = Vision_Enter_Yaw + g_project_one_yaw;
 *
 * 注意：
 *      本模块只负责输出视觉相对 yaw，不直接操作电机
 *
 *********************************************************************************************************************/


//-------------------------------------------------------------------------------------------------------------------
// 图像中心
//-------------------------------------------------------------------------------------------------------------------
#define PROJECT_ONE_IMAGE_CENTER_X         (PROJECT_ONE_IMAGE_W / 2)


//-------------------------------------------------------------------------------------------------------------------
// ROI 区域
// 只在图像中下部搜索锥桶，减小远处噪声干扰
//-------------------------------------------------------------------------------------------------------------------
#define PROJECT_ONE_ROI_Y_START            (45)
#define PROJECT_ONE_ROI_Y_END              (119)


//-------------------------------------------------------------------------------------------------------------------
// 锥桶块提取参数
//-------------------------------------------------------------------------------------------------------------------
#define PROJECT_ONE_COLUMN_HIT_THRESHOLD   (8)     // 一列中“障碍像素”超过该值，则视为该列属于锥桶块
#define PROJECT_ONE_MIN_BLOCK_WIDTH        (4)     // 锥桶块最小宽度
#define PROJECT_ONE_MAX_BLOCK_NUM          (8)     // 最多保留的锥桶块数量


//-------------------------------------------------------------------------------------------------------------------
// 安全边距参数
// 锥桶越近，安全边距越大
//-------------------------------------------------------------------------------------------------------------------
#define PROJECT_ONE_MARGIN_MIN             (8)
#define PROJECT_ONE_MARGIN_MID             (12)
#define PROJECT_ONE_MARGIN_MAX             (18)


//-------------------------------------------------------------------------------------------------------------------
// 偏差转 yaw 使用的参考宽度
//-------------------------------------------------------------------------------------------------------------------
#define PROJECT_ONE_TRACK_WIDTH            (60)


//-------------------------------------------------------------------------------------------------------------------
// 丢失目标相关参数
//-------------------------------------------------------------------------------------------------------------------
#define PROJECT_ONE_LOST_KEEP_FRAME        (5)     // 短时丢失保持上一帧目标的帧数
#define PROJECT_ONE_SCAN_STEP_NUM          (6)     // 搜索表长度


//-------------------------------------------------------------------------------------------------------------------
// 搜索 yaw 表
// 含义：当视觉连续丢失锥桶时，输出一组小幅摆扫角，让车头主动去找锥桶
//
// 右优先：先向右扫，再向左扫
// 左优先：先向左扫，再向右扫
//-------------------------------------------------------------------------------------------------------------------
static const float g_project_one_scan_table_right_first[PROJECT_ONE_SCAN_STEP_NUM] = { 8.0f, 15.0f, 22.0f, -8.0f, -15.0f, -22.0f };
static const float g_project_one_scan_table_left_first [PROJECT_ONE_SCAN_STEP_NUM] = {-8.0f,-15.0f,-22.0f,  8.0f,  15.0f,  22.0f };


//-------------------------------------------------------------------------------------------------------------------
// 锥桶块结构体
//-------------------------------------------------------------------------------------------------------------------
typedef struct
{
    int x_start;        // 锥桶块左边界
    int x_end;          // 锥桶块右边界
    int x_center;       // 锥桶块中心
    int width;          // 锥桶块宽度
    int strength;       // 块强度：列响应总和，越大通常越可靠
    int height_hint;    // 接近程度：列最大响应，越大说明障碍越近
} project_one_block_struct;


//-------------------------------------------------------------------------------------------------------------------
// 全局变量定义
//-------------------------------------------------------------------------------------------------------------------
uint8 g_project_one_binary_image[PROJECT_ONE_IMAGE_H][PROJECT_ONE_IMAGE_W];
int   g_project_one_target_x = PROJECT_ONE_IMAGE_CENTER_X;
int   g_project_one_error = 0;
float g_project_one_yaw = 0.0f;
uint8 g_project_one_enable_flag = 0;
uint8 g_project_one_block_count = 0;
uint8 g_project_one_state = PROJECT_ONE_VISION_DISABLE;
uint8 g_project_one_lost_frame_count = 0;


//-------------------------------------------------------------------------------------------------------------------
// 内部静态变量
//-------------------------------------------------------------------------------------------------------------------
static project_one_block_struct s_blocks[PROJECT_ONE_MAX_BLOCK_NUM];
static int   s_last_target_x = PROJECT_ONE_IMAGE_CENTER_X;   // 上一帧目标点
static int   s_last_valid_direction = 0;                     // 上一次有效绕行方向：-1 左，1 右，0 未知
static uint8 s_scan_step = 0;                                // 当前搜索步号


//-------------------------------------------------------------------------------------------------------------------
// 内部函数声明
//-------------------------------------------------------------------------------------------------------------------
static void  project_one_clear_blocks(void);
static void  project_one_find_blocks(const uint8 binary[PROJECT_ONE_IMAGE_H][PROJECT_ONE_IMAGE_W]);
static void  project_one_sort_blocks_by_strength(void);
static int   project_one_calc_margin(const project_one_block_struct *block);
static int   project_one_calc_target_x(void);
static int   project_one_clamp(int value, int min_value, int max_value);
static float project_one_get_scan_yaw(void);


/*********************************************************************************************************************
 * 函数名称：project_one_vision_init
 * 函数功能：初始化科目一摄像头绕桩模块
 *
 * 作用：
 * 1. 清空所有视觉输出变量
 * 2. 复位状态机
 * 3. 初始化 auto_binary 模块
 *
 *********************************************************************************************************************/
void project_one_vision_init(void)
{
    g_project_one_target_x = PROJECT_ONE_IMAGE_CENTER_X;
    g_project_one_error = 0;
    g_project_one_yaw = 0.0f;
    g_project_one_enable_flag = 0;
    g_project_one_block_count = 0;
    g_project_one_state = PROJECT_ONE_VISION_DISABLE;
    g_project_one_lost_frame_count = 0;

    s_last_target_x = PROJECT_ONE_IMAGE_CENTER_X;
    s_last_valid_direction = 0;
    s_scan_step = 0;

    auto_process_init();
}


/*********************************************************************************************************************
 * 函数名称：project_one_vision_enable
 * 函数功能：启用绕桩视觉模块
 *
 * 说明：
 * 该函数通常在 GPS 判断“已接近绕桩区”时调用
 *
 *********************************************************************************************************************/
void project_one_vision_enable(void)
{
    g_project_one_enable_flag = 1;
    g_project_one_state = PROJECT_ONE_VISION_SEARCH;
    g_project_one_lost_frame_count = 0;
    s_scan_step = 0;
}


/*********************************************************************************************************************
 * 函数名称：project_one_vision_disable
 * 函数功能：关闭绕桩视觉模块
 *
 *********************************************************************************************************************/
void project_one_vision_disable(void)
{
    g_project_one_enable_flag = 0;
    g_project_one_state = PROJECT_ONE_VISION_DISABLE;
    g_project_one_lost_frame_count = 0;
    s_scan_step = 0;
}


/*********************************************************************************************************************
 * 函数名称：project_one_vision_run
 * 函数功能：每帧执行一次绕桩视觉处理
 *
 * 处理逻辑：
 * 1. auto_process()：自动曝光 + 自动阈值 + 二值化
 * 2. project_one_find_blocks()：提取锥桶块
 * 3. 若提取到锥桶块：
 *      - 单桶：向远离锥桶的一侧绕
 *      - 双桶：取安全通道中心
 * 4. 若未提取到锥桶块：
 *      - 短时丢失：保持上一帧目标
 *      - 持续丢失：输出摆扫搜索 yaw
 *
 *********************************************************************************************************************/
void project_one_vision_run(const uint8 image[PROJECT_ONE_IMAGE_H][PROJECT_ONE_IMAGE_W])
{
    int target_x;

    if(!g_project_one_enable_flag)
    {
        return;
    }

    // 1. 自动曝光 + 自动阈值 + 自动二值化
    auto_process(image, g_project_one_binary_image);

    // 2. 从二值图中提取锥桶块
    project_one_find_blocks(g_project_one_binary_image);

    // 3. 根据识别情况决定输出
    if(g_project_one_block_count > 0)
    {
        // 重新识别到锥桶，退出丢失/搜索状态
        g_project_one_lost_frame_count = 0;
        s_scan_step = 0;

        target_x = project_one_calc_target_x();
        g_project_one_target_x = target_x;
        g_project_one_error = target_x - PROJECT_ONE_IMAGE_CENTER_X;
        g_project_one_yaw = offset_to_yaw(g_project_one_error);
    }
    else
    {
        // 没看到锥桶
        g_project_one_lost_frame_count++;

        if(g_project_one_lost_frame_count <= PROJECT_ONE_LOST_KEEP_FRAME)
        {
            // 短时丢失：先沿用上一帧方向，避免一丢目标就乱摆
            g_project_one_state = PROJECT_ONE_VISION_LOST;
            g_project_one_target_x = s_last_target_x;
            g_project_one_error = s_last_target_x - PROJECT_ONE_IMAGE_CENTER_X;
            g_project_one_yaw = offset_to_yaw(g_project_one_error);
        }
        else
        {
            // 持续丢失：主动进入摆扫搜索
            g_project_one_state = PROJECT_ONE_VISION_SCAN;
            g_project_one_target_x = s_last_target_x;
            g_project_one_error = s_last_target_x - PROJECT_ONE_IMAGE_CENTER_X;
            g_project_one_yaw = project_one_get_scan_yaw();
        }
    }
}


/*********************************************************************************************************************
 * 函数名称：project_one_clear_blocks
 * 函数功能：清空当前帧锥桶块缓存
 *
 *********************************************************************************************************************/
static void project_one_clear_blocks(void)
{
    uint8 i;

    g_project_one_block_count = 0;

    for(i = 0; i < PROJECT_ONE_MAX_BLOCK_NUM; i++)
    {
        s_blocks[i].x_start = 0;
        s_blocks[i].x_end = 0;
        s_blocks[i].x_center = 0;
        s_blocks[i].width = 0;
        s_blocks[i].strength = 0;
        s_blocks[i].height_hint = 0;
    }
}


/*********************************************************************************************************************
 * 函数名称：project_one_find_blocks
 * 函数功能：从二值图中提取锥桶块
 *
 * 输入：
 *      binary：二值图
 *
 * 实现思路：
 * 1. 在 ROI 区域统计每一列“障碍像素”个数
 * 2. 如果某列障碍像素数超过阈值，则视为该列属于锥桶
 * 3. 将连续满足条件的列合并成一个锥桶块
 *
 * 重要说明：
 * 当前默认“黑色 = 障碍”，因此判断条件写成：
 *      if(binary[y][x] == 0)
 *
 * 如果你们现场二值极性相反，把它改为：
 *      if(binary[y][x] == 255)
 *
 *********************************************************************************************************************/
static void project_one_find_blocks(const uint8 binary[PROJECT_ONE_IMAGE_H][PROJECT_ONE_IMAGE_W])
{
    int col_sum[PROJECT_ONE_IMAGE_W] = {0};
    int x, y;
    int in_block = 0;
    int start_x = 0;

    project_one_clear_blocks();

    // 1. 统计每一列障碍像素数量
    for(x = 0; x < PROJECT_ONE_IMAGE_W; x++)
    {
        int sum = 0;

        for(y = PROJECT_ONE_ROI_Y_START; y <= PROJECT_ONE_ROI_Y_END; y++)
        {
            if(binary[y][x] == 0)
            {
                sum++;
            }
        }

        col_sum[x] = sum;
    }

    // 2. 将连续高响应列合并成块
    for(x = 0; x < PROJECT_ONE_IMAGE_W; x++)
    {
        if(col_sum[x] >= PROJECT_ONE_COLUMN_HIT_THRESHOLD)
        {
            if(!in_block)
            {
                in_block = 1;
                start_x = x;
            }
        }
        else
        {
            if(in_block)
            {
                int end_x = x - 1;
                int width = end_x - start_x + 1;

                if(width >= PROJECT_ONE_MIN_BLOCK_WIDTH && g_project_one_block_count < PROJECT_ONE_MAX_BLOCK_NUM)
                {
                    int k;
                    int strength = 0;
                    int max_col = 0;
                    uint8 idx = g_project_one_block_count;

                    for(k = start_x; k <= end_x; k++)
                    {
                        strength += col_sum[k];
                        if(col_sum[k] > max_col)
                        {
                            max_col = col_sum[k];
                        }
                    }

                    s_blocks[idx].x_start = start_x;
                    s_blocks[idx].x_end = end_x;
                    s_blocks[idx].x_center = (start_x + end_x) / 2;
                    s_blocks[idx].width = width;
                    s_blocks[idx].strength = strength;
                    s_blocks[idx].height_hint = max_col;

                    g_project_one_block_count++;
                }

                in_block = 0;
            }
        }
    }

    // 3. 处理最后一个块
    if(in_block)
    {
        int end_x = PROJECT_ONE_IMAGE_W - 1;
        int width = end_x - start_x + 1;

        if(width >= PROJECT_ONE_MIN_BLOCK_WIDTH && g_project_one_block_count < PROJECT_ONE_MAX_BLOCK_NUM)
        {
            int k;
            int strength = 0;
            int max_col = 0;
            uint8 idx = g_project_one_block_count;

            for(k = start_x; k <= end_x; k++)
            {
                strength += col_sum[k];
                if(col_sum[k] > max_col)
                {
                    max_col = col_sum[k];
                }
            }

            s_blocks[idx].x_start = start_x;
            s_blocks[idx].x_end = end_x;
            s_blocks[idx].x_center = (start_x + end_x) / 2;
            s_blocks[idx].width = width;
            s_blocks[idx].strength = strength;
            s_blocks[idx].height_hint = max_col;

            g_project_one_block_count++;
        }
    }

    // 4. 按强度排序，优先使用更可靠的锥桶块
    project_one_sort_blocks_by_strength();
}


/*********************************************************************************************************************
 * 函数名称：project_one_sort_blocks_by_strength
 * 函数功能：按锥桶块强度从大到小排序
 *
 * 作用：
 *      优先使用更明显、更可靠的块进行决策
 *
 *********************************************************************************************************************/
static void project_one_sort_blocks_by_strength(void)
{
    uint8 i, j;

    for(i = 0; i < g_project_one_block_count; i++)
    {
        for(j = i + 1; j < g_project_one_block_count; j++)
        {
            if(s_blocks[j].strength > s_blocks[i].strength)
            {
                project_one_block_struct temp = s_blocks[i];
                s_blocks[i] = s_blocks[j];
                s_blocks[j] = temp;
            }
        }
    }
}


/*********************************************************************************************************************
 * 函数名称：project_one_calc_margin
 * 函数功能：根据锥桶大小/接近程度计算安全边距
 *
 * 输入：
 *      block：锥桶块
 *
 * 输出：
 *      返回对应的安全边距
 *
 * 设计思路：
 *      锥桶越近，图像中宽度越大、高度响应越大，需要更大的安全距离
 *
 *********************************************************************************************************************/
static int project_one_calc_margin(const project_one_block_struct *block)
{
    if(block->height_hint > 40 || block->width > 25)
    {
        return PROJECT_ONE_MARGIN_MAX;
    }
    else if(block->height_hint > 20 || block->width > 12)
    {
        return PROJECT_ONE_MARGIN_MID;
    }
    else
    {
        return PROJECT_ONE_MARGIN_MIN;
    }
}


/*********************************************************************************************************************
 * 函数名称：project_one_clamp
 * 函数功能：对数值进行限幅
 *
 *********************************************************************************************************************/
static int project_one_clamp(int value, int min_value, int max_value)
{
    if(value < min_value)
    {
        value = min_value;
    }
    if(value > max_value)
    {
        value = max_value;
    }
    return value;
}


/*********************************************************************************************************************
 * 函数名称：project_one_calc_target_x
 * 函数功能：根据当前识别到的锥桶块，计算本帧目标点列坐标
 *
 * 处理策略：
 * 1. 两个及以上锥桶块：
 *      优先取最强两个块
 *      使用“边界 + 安全距离”求安全通道中心
 *
 * 2. 仅一个锥桶块：
 *      向远离锥桶的一侧绕行
 *
 * 3. 内部同时维护：
 *      s_last_target_x
 *      s_last_valid_direction
 *
 * 这些变量用于后续丢目标时继续保持方向
 *
 *********************************************************************************************************************/
static int project_one_calc_target_x(void)
{
    int target_x = s_last_target_x;

    if(g_project_one_block_count >= 2)
    {
        project_one_block_struct *block_a = &s_blocks[0];
        project_one_block_struct *block_b = &s_blocks[1];
        project_one_block_struct *left_block;
        project_one_block_struct *right_block;

        int margin_left;
        int margin_right;
        int safe_left;
        int safe_right;

        // 先区分左右块
        if(block_a->x_center < block_b->x_center)
        {
            left_block = block_a;
            right_block = block_b;
        }
        else
        {
            left_block = block_b;
            right_block = block_a;
        }

        margin_left = project_one_calc_margin(left_block);
        margin_right = project_one_calc_margin(right_block);

        // 用“边界”而不是“中心”计算安全通道
        safe_left = left_block->x_end + margin_left;
        safe_right = right_block->x_start - margin_right;

        if(safe_left < safe_right)
        {
            // 正常双桶通道
            target_x = (safe_left + safe_right) / 2;
            g_project_one_state = PROJECT_ONE_VISION_DOUBLE_CONE;

            if(target_x < PROJECT_ONE_IMAGE_CENTER_X)
            {
                s_last_valid_direction = -1;
            }
            else if(target_x > PROJECT_ONE_IMAGE_CENTER_X)
            {
                s_last_valid_direction = 1;
            }
        }
        else
        {
            // 安全通道太窄，退化为绕行更强的那个块
            if(left_block->strength >= right_block->strength)
            {
                target_x = left_block->x_end + project_one_calc_margin(left_block);
                s_last_valid_direction = 1;
            }
            else
            {
                target_x = right_block->x_start - project_one_calc_margin(right_block);
                s_last_valid_direction = -1;
            }

            g_project_one_state = PROJECT_ONE_VISION_SINGLE_CONE;
        }
    }
    else if(g_project_one_block_count == 1)
    {
        project_one_block_struct *block = &s_blocks[0];
        int margin = project_one_calc_margin(block);

        // 单桶时不要去桶中心，而是向它的反方向绕
        if(block->x_center < PROJECT_ONE_IMAGE_CENTER_X)
        {
            // 锥桶在左边，往右绕
            target_x = block->x_end + margin;
            s_last_valid_direction = 1;
        }
        else
        {
            // 锥桶在右边，往左绕
            target_x = block->x_start - margin;
            s_last_valid_direction = -1;
        }

        g_project_one_state = PROJECT_ONE_VISION_SINGLE_CONE;
    }

    target_x = project_one_clamp(target_x, 0, PROJECT_ONE_IMAGE_W - 1);
    s_last_target_x = target_x;

    return target_x;
}


/*********************************************************************************************************************
 * 函数名称：project_one_get_scan_yaw
 * 函数功能：获取当前搜索状态下的摆扫 yaw
 *
 * 设计思路：
 * 1. 如果上一次有效方向是向右，则优先先向右扫
 * 2. 如果上一次有效方向是向左，则优先先向左扫
 * 3. 扫描角逐步增大，再扫回另一侧
 *
 * 目的：
 *      当 GPS 有偏差、摄像头暂时没看到锥桶时，让车头主动找桶
 *
 *********************************************************************************************************************/
static float project_one_get_scan_yaw(void)
{
    float yaw_out = 0.0f;

    if(s_last_valid_direction >= 0)
    {
        yaw_out = g_project_one_scan_table_right_first[s_scan_step];
    }
    else
    {
        yaw_out = g_project_one_scan_table_left_first[s_scan_step];
    }

    s_scan_step++;
    if(s_scan_step >= PROJECT_ONE_SCAN_STEP_NUM)
    {
        s_scan_step = 0;
    }

    return yaw_out;
}