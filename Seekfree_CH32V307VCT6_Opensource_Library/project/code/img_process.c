/**
 * @file img_process.c
 * @author jyh
 * @brief 
 * @date 2026-07-20
 * @function: 图像处理
 * @copyright Copyright (c) 2026
 * 
 */

#include "zf_common_headfile.h"
#include "img_process.h"
#include "cam_preprocess.h"
#include  <math.h>

#define CORNER_STABLE_THRESHOLD      12
#define CORNER_CHANGE_2_THRESHOLD    3
#define CORNER_CHANGE_3_THRESHOLD    5
#define CORNER_CHANGE_4_THRESHOLD    6
#define CORNER_UP_MAX_ROW_DIFF       28
#define CROSS_BOTH_LOST_MIN          2



//用户函数声明区
extern const uint8 Image_Flags[][9][8];     //数字图上的数字表(本工程未使用)
volatile int Left_Line[IMG_ROW]; //左边界数组
volatile int Right_Line[IMG_ROW];//右边界数组
volatile int Mid_Line[IMG_ROW];  //中线数组
volatile int Road_Wide[IMG_ROW]; //赛道宽度
volatile int White_Column[IMG_COL];    //每列白列长度(按列索引)
volatile int Search_Stop_Line;     //搜索截止行,只记录长度,要用的时候用 IMG_ROW 减掉这个值
volatile int Boundry_Start_Left;   //左边界起始行
volatile int Boundry_Start_Right;  //右边界起始行
volatile int Left_Lost_Time;       //左边界丢线数
volatile int Right_Lost_Time;
volatile int Both_Lost_Time;//两边同时丢线数
int Longest_White_Column_Left[2]; //最长白列,[0]是最长白列的长度,也就是Search_Stop_Line搜索截止行,[1]是第某列
int Longest_White_Column_Right[2];//最长白列,[0]是最长白列的长度,也就是Search_Stop_Line搜索截止行,[1]是第某列
int Left_Lost_Flag[IMG_ROW] ; //左丢线数组,丢线为1,没丢为0
int Right_Lost_Flag[IMG_ROW]; //右丢线数组,丢线为1,没丢为0

//环岛
extern volatile int Island_State;     //环岛状态标志
extern volatile int Left_Island_Flag; //左环岛标志
extern volatile int Right_Island_Flag;//右环岛标志
//坡道
volatile int Ramp_Flag=0;//坡道标志
//斑马线
volatile int Zebra_Stripes_Flag=0;//斑马线
//图像丢失
volatile uint8 Img_Disappear_Flag=0; //图像丢失标志位,1说明丢图
volatile uint8 Outside_Edge_flag=0;  //超出边界标志,用不到可以删掉
volatile uint8 Stop_Flag=0;          //停止标志
//十字
volatile int Cross_Flag=0;
volatile int Left_Down_Find=0; //十字使用,找到的下拐点行号,没找到就是0
volatile int Left_Up_Find=0;   //四个拐点的标志
volatile int Right_Down_Find=0;
volatile int Right_Up_Find=0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     双最长白列巡线
  @param     null
  @return    null
  Sample     Longest_White_Column();
  @note      最长白列巡线,寻找出起始边界,边线,最长白列等基础元素,
             后面提取这些数据用于其他元素识别
-------------------------------------------------------------------------------------------------------------------*/
void Longest_White_Column(void) // 最长白列巡线
{
    int i, j;

    int start_column = 0;            // 最长白列的搜索起点列
    int end_column   = IMG_COL - 0;

    int left_border  = 0;
    int right_border = IMG_COL - 1;   // 临时存储右边界位置

    Longest_White_Column_Left[0]  = 0; // [0]是最长白列的长度
    Longest_White_Column_Left[1]  = 0; // [1]是最长白列所在的列

    Longest_White_Column_Right[0] = 0; // [0]是最长白列的长度
    Longest_White_Column_Right[1] = 0; // [1]是最长白列所在的列

    Right_Lost_Time = 0;              // 边界丢线数
    Left_Lost_Time  = 0;

    Boundry_Start_Left  = 0;          // 第一行是丢线点,左边界起始行
    Boundry_Start_Right = 0;

    Both_Lost_Time = 0;               // 两边同时丢线数

    /*
     * ★ 环岛状态3: 锁定最长白列的搜索范围 ★
     *
     * (原工程这里就有这段逻辑, 只是被注释掉了, 数值按 188 -> 94 减半)
     *
     * 为什么需要:
     *   start_column / end_column 不只决定 Search_Stop_Line,
     *   它还是"边界搜索的种子列" —— 下面提边界时,
     *       Longest_White_Column_Right[1] 是往右找右边界的起点,
     *       Longest_White_Column_Left[1]  是往左找左边界的起点。
     *   车开到环岛口时, 最长白列很可能落在图像左侧(穿过环岛开口),
     *   种子列跟着跑偏, 提出来的"边界"就不是环岛外沿了, 角点判据也就废了。
     *   把列范围限制到右侧, 种子列被逼到环岛那一侧, 信号就干净了。
     *
     * 什么时候失效:
     *   靠 Island_State 判断, 状态3 一结束自动恢复全图搜索, 不用手动清。
     */
    if (Island_State == 3)
    {
        if (Right_Island_Flag == 1)                     /* 右环岛: 锁图像最右侧 */
        {
            start_column = ISL3_S3_COL_R_START;
            end_column   = ISL3_S3_COL_R_END;
        }
        else if (Left_Island_Flag == 1)                 /* 左环岛: 锁最左侧(镜像) */
        {
            start_column = ISL3_S3_COL_L_START;
            end_column   = ISL3_S3_COL_L_END;
        }
    }

    /*
     * 防止 IMG_COL 太小时搜索范围溢出
     * 下面需要访问 j-2 和 j+2,至少要保证不越界
     */
    if (start_column < 2)
    {
        start_column = 2;
    }

    if (end_column > IMG_COL - 3)
    {
        end_column = IMG_COL - 3;
    }

    if (start_column > end_column)
    {
        start_column = 2;
        end_column   = IMG_COL - 3;
    }

    // 搜索范围
    for (i = 0; i < IMG_ROW; i++)
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i]  = 0;

        Left_Line[i]  = 0;
        Right_Line[i] = IMG_COL - 1;

        /* 边界数组整个重建了, 上一帧的补线标记跟着作废。
           环岛补线(Isl_Patch_Mark)的标记就在这儿清 —— 详见 island.c 的说明 */
        Isl_Patched[i] = 0;
    }

    for (i = 0; i < IMG_COL; i++)
    {
        White_Column[i] = 0;
    }

// 原来这里有一段注释掉的"第3状态改变最长白列寻找范围"的代码,
// 已经挪到本函数开头启用了(在防止越界的钳位之前, 这样种子列从源头就变了)。
// 具体数值见 island.h 的 ISL3_S3_COL_* 宏。

    /*
     * 从左,右两边向上遍历图像, 记录每一列底部连续白的个数。
     *
     * ★ 这里始终扫【全图】, 不受 start_column/end_column 限制 ★
     *
     *   White_Column[] 有两个用途:
     *       ① 选边界搜索的种子列  —— 这个要受锁定范围限制
     *       ② 算 Search_Stop_Line —— 它是"边界数据有效到第几行"的度量,
     *                                绝对不能受锁定影响
     *
     *   这两件事原来共用同一次扫描, 于是环岛状态3 一把锁定打开,
     *   Search_Stop_Line 就变成"锁定范围里最长的白列"。而锁定范围是图像
     *   最右侧那一小段, 环岛口那几列往往很短 —— SS 直接塌掉。
     *   SS 一塌, Find_*_Up_Point 里那句
     *       if (end <= IMG_ROW - Search_Stop_Line) end = IMG_ROW - Search_Stop_Line;
     *   就会把 end 抬到起始行之上, 角点循环一次都不进 -> 返回 0 ->
     *   状态3 一帧补线都画不出来。
     *
     *   现在扫描范围和锁定解耦: 全图扫一遍, 只有下面"选种子列"那两步
     *   才按 [start_column, end_column] 过滤。
     *   (顺带一提, 没锁定的时候本来就是这个范围, 所以平时开销没变。)
     */
    for (j = 2; j <= IMG_COL - 3; j++)
    {
        for (i = IMG_ROW - 1; i >= 0; i--)
        {
            if (BinaryImg_CDM[i][j] == IMG_BLACK)
            {
                break;
            }
            else
            {
                White_Column[j]++;
            }
        }
    }

    // 寻找左边的最长白列
    Longest_White_Column_Left[0] = 0;

    for (i = start_column; i <= end_column; i++)
    {
        if (Longest_White_Column_Left[0] < White_Column[i])
        {
            Longest_White_Column_Left[0] = White_Column[i]; // [0]白列长度
            Longest_White_Column_Left[1] = i;               // [1]所在列
        }
    }

    // 再找右边的最长白列
    Longest_White_Column_Right[0] = 0;

    for (i = end_column; i >= start_column; i--)
    {
        if (Longest_White_Column_Right[0] < White_Column[i])
        {
            Longest_White_Column_Right[0] = White_Column[i]; // [0]白列长度
            Longest_White_Column_Right[1] = i;               // [1]所在列
        }
    }

    /*
     * 搜索截止行选取的就是最长白列长度
     * 如果最长白列过长就取上限
     *
     * ★ 这里必须重新在【全图】上取一次最大值, 不能用
     *   Longest_White_Column_Left[0] ★
     *   那个 [0] 是在锁定范围里选出来的(见上面两步), 拿它当 SS 会被锁定带偏,
     *   环岛状态3 时 SS 会莫名其妙变小, 后面的角点判据跟着全废。
     *   [1] 才是种子的列号, 那个受锁定限制是对的。
     */
    Search_Stop_Line = 0;

    for (i = 2; i <= IMG_COL - 3; i++)
    {
        if (Search_Stop_Line < White_Column[i])
        {
            Search_Stop_Line = White_Column[i];
        }
    }

    if (Search_Stop_Line > IMG_ROW)
    {
        Search_Stop_Line = IMG_ROW;
    }

    // 开始巡线
    for (i = IMG_ROW - 1;
         i >= IMG_ROW - Search_Stop_Line;
         i--)
    {
        /*
         * 每一行往下更新时边界默认值
         * 如果本行未找到边界就用上一行的结果
         */
        left_border  = 0;
        right_border = IMG_COL - 1;

        /*
         * 从右往左寻找右边线
         */
        for (j = Longest_White_Column_Right[1];
             j <= IMG_COL - 3;
             j++)
        {
            /*
             * 白黑跳变,认为是找到了右边界
             */
            if (BinaryImg_CDM[i][j]     == IMG_WHITE &&
                BinaryImg_CDM[i][j + 1] == IMG_BLACK &&
                BinaryImg_CDM[i][j + 2] == IMG_BLACK)
            {
                right_border = j;
                Right_Lost_Flag[i] = 0; // 未丢线
                break;
            }
            else if (j >= IMG_COL - 3)
            {
                /*
                 * 搜索到图像右边还未找到边界,
                 * 把图像最右边作为右边界
                 */
                right_border = IMG_COL - 1;
                Right_Lost_Flag[i] = 1; // 右边丢线
                break;
            }
        }

        /*
         * 再从左往右寻找左边线
         */
        for (j = Longest_White_Column_Left[1];
             j >= 2;
             j--)
        {
            /*
             * 白黑跳变,认为是当前行的左边界
             */
            if (BinaryImg_CDM[i][j]     == IMG_WHITE &&
                BinaryImg_CDM[i][j - 1] == IMG_BLACK &&
                BinaryImg_CDM[i][j - 2] == IMG_BLACK)
            {
                left_border = j;
                Left_Lost_Flag[i] = 0; // 未丢线
                break;
            }
            else if (j <= 2)
            {
                /*
                 * 搜索到图像左边还未找到边界,
                 * 把图像最左边作为左边界
                 */
                left_border = 0;
                Left_Lost_Flag[i] = 1; // 左边丢线
                break;
            }
        }

        Left_Line[i]  = left_border;
        Right_Line[i] = right_border;
    }

    // 补充特殊情况的边界
    for (i = IMG_ROW - 1; i >= 0; i--)
    {
        // 单边丢线
        if (Left_Lost_Flag[i] == 1)
        {
            Left_Lost_Time++;
        }

        if (Right_Lost_Flag[i] == 1)
        {
            Right_Lost_Time++;
        }

        // 双边丢线
        if (Left_Lost_Flag[i] == 1 &&
            Right_Lost_Flag[i] == 1)
        {
            Both_Lost_Time++;
        }

        // 记录第一行是丢线点,边界起始行
        if (Boundry_Start_Left == 0 &&
            Left_Lost_Flag[i] != 1)
        {
            Boundry_Start_Left = i;
        }

        if (Boundry_Start_Right == 0 &&
            Right_Lost_Flag[i] != 1)
        {
            Boundry_Start_Right = i;
        }

        Road_Wide[i] = Right_Line[i] - Left_Line[i];
    }

// 第3状态改边界,这里原来是为环岛准备的,现在不需要这些变量了
//    if (Island_State == 3 || Island_State == 4)
//    {
//        if (Right_Island_Flag == 1) // 右环
//        {
//            for (i = IMG_ROW - 1; i >= 0; i--)
//            {
//                // 右边界直接写死到图像最右边
//                Right_Line[i] = IMG_COL - 1;
//            }
//        }
//        else if (Left_Island_Flag == 1) // 左环
//        {
//            for (i = IMG_ROW - 1; i >= 0; i--)
//            {
//                // 左边界直接写死到图像最左边
//                Left_Line[i] = 0;
//            }
//        }
//    }

    // debug使用,屏幕显示相关变量
//    ips200_showint16(0, 0, Longest_White_Column_Right[0]); // [0]白列长度
//    ips200_showint16(0, 1, Longest_White_Column_Right[1]); // [1]所在列
//    ips200_showint16(0, 2, Longest_White_Column_Left[0]);  // [0]白列长度
//    ips200_showint16(0, 3, Longest_White_Column_Left[1]);  // [1]所在列
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左补线
  @param     起始点坐标,终点坐标
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      补出来的线是边界,可信度高的话还要再补
-------------------------------------------------------------------------------------------------------------------*/
void Left_Add_Line(int x1, int y1, int x2, int y2) // 左补线,补的是边界
{
    int i, max, a1, a2;
    int hx;

    /* ★ 把这几行标成"补过的" ★
       下面写进去的 Left_Line[] 会被 Left_Lost_Flag 那道闸门挡住 ——
       Calculate_Mid_Line 和 TFT180_Show_Track_Lines 都是看那个标志决定
       用不用这一行。不在这里开门的话, 补线就是白补。
       详见 island.c 里 Isl_Patched 的说明。 */
    Isl_Patch_Mark(y1, y2);

    // 起始点位置校正,防止出现越界的情况
    if (x1 >= IMG_COL - 1)
    {
        x1 = IMG_COL - 1;
    }
    else if (x1 <= 0)
    {
        x1 = 0;
    }

    if (y1 >= IMG_ROW - 1)
    {
        y1 = IMG_ROW - 1;
    }
    else if (y1 <= 0)
    {
        y1 = 0;
    }

    if (x2 >= IMG_COL - 1)
    {
        x2 = IMG_COL - 1;
    }
    else if (x2 <= 0)
    {
        x2 = 0;
    }

    if (y2 >= IMG_ROW - 1)
    {
        y2 = IMG_ROW - 1;
    }
    else if (y2 <= 0)
    {
        y2 = 0;
    }

    /*
     * 两点处于同一行时无法求出 y 的差值,
     * 防止除以零
     */
    if (y1 == y2)
    {
        Left_Line[y1] = x1;
        return;
    }

    a1 = y1;
    a2 = y2;

    if (a1 > a2) // 坐标互换
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }

    for (i = a1; i <= a2; i++) // 按斜率插值补线
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;

        if (hx >= IMG_COL)
        {
            hx = IMG_COL - 1;
        }
        else if (hx <= 0)
        {
            hx = 0;
        }

        Left_Line[i] = hx;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右补线
  @param     起始点坐标,终点坐标
  @return    null
  Sample     Right_Add_Line(int x1,int y1,int x2,int y2);
  @note      补出来的线是边界,可信度高的话还要再补
-------------------------------------------------------------------------------------------------------------------*/
void Right_Add_Line(int x1, int y1, int x2, int y2) // 右补线,补的是边界
{
    int i, max, a1, a2;
    int hx;

    /* 同 Left_Add_Line: 不标记的话补线会被丢线标志挡住, 等于白补 */
    Isl_Patch_Mark(y1, y2);

    // 起始点位置校正,防止出现越界的情况
    if (x1 >= IMG_COL - 1)
    {
        x1 = IMG_COL - 1;
    }
    else if (x1 <= 0)
    {
        x1 = 0;
    }

    if (y1 >= IMG_ROW - 1)
    {
        y1 = IMG_ROW - 1;
    }
    else if (y1 <= 0)
    {
        y1 = 0;
    }

    if (x2 >= IMG_COL - 1)
    {
        x2 = IMG_COL - 1;
    }
    else if (x2 <= 0)
    {
        x2 = 0;
    }

    if (y2 >= IMG_ROW - 1)
    {
        y2 = IMG_ROW - 1;
    }
    else if (y2 <= 0)
    {
        y2 = 0;
    }

    /*
     * 两点处于同一行时无法求出 y 的差值,
     * 防止除以零
     */
    if (y1 == y2)
    {
        Right_Line[y1] = x1;
        return;
    }

    a1 = y1;
    a2 = y2;

    if (a1 > a2) // 坐标互换
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }

    for (i = a1; i <= a2; i++) // 按斜率插值补线
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;

        if (hx >= IMG_COL)
        {
            hx = IMG_COL - 1;
        }
        else if (hx <= 0)
        {
            hx = 0;
        }

        Right_Line[i] = hx;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     寻找下拐点,十字使用
  @param     搜索范围的起点行,终点行
  @return    修改后的全局拐点
             Right_Down_Find=0;
             Left_Down_Find=0;
  Sample     Find_Down_Point(int start,int end)
  @note      使用之前先查看对应的变量与注释,没找到时对应的变量置0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Down_Point(int start, int end)
{
    int i, t;

    Right_Down_Find = 0;
    Left_Down_Find = 0;

    if(start < end)
    {
        t = start;
        start = end;
        end = t;
    }

    // 这里需要访问 i+3,所以底部最多到倒数第4行
    if(start > IMG_ROW - 4)
    {
        start = IMG_ROW - 4;
    }

    if(end < IMG_ROW - Search_Stop_Line)
    {
        end = IMG_ROW - Search_Stop_Line;
    }

    // 这里需要访问 i-4,所以最小只能到第4行
    if(end < 4)
    {
        end = 4;
    }

    for(i = start; i >= end; i--)
    {
        if(Left_Down_Find == 0 &&
           abs(Left_Line[i] - Left_Line[i + 1]) <= CORNER_STABLE_THRESHOLD &&
           abs(Left_Line[i + 1] - Left_Line[i + 2]) <= CORNER_STABLE_THRESHOLD &&
           abs(Left_Line[i + 2] - Left_Line[i + 3]) <= CORNER_STABLE_THRESHOLD &&
           Left_Line[i] - Left_Line[i - 2] >= CORNER_CHANGE_2_THRESHOLD &&
           Left_Line[i] - Left_Line[i - 3] >= CORNER_CHANGE_3_THRESHOLD &&
           Left_Line[i] - Left_Line[i - 4] >= CORNER_CHANGE_4_THRESHOLD)
        {
            Left_Down_Find = i;
        }

        if(Right_Down_Find == 0 &&
           abs(Right_Line[i] - Right_Line[i + 1]) <= CORNER_STABLE_THRESHOLD &&
           abs(Right_Line[i + 1] - Right_Line[i + 2]) <= CORNER_STABLE_THRESHOLD &&
           abs(Right_Line[i + 2] - Right_Line[i + 3]) <= CORNER_STABLE_THRESHOLD &&
           Right_Line[i] - Right_Line[i - 2] <= -CORNER_CHANGE_2_THRESHOLD &&
           Right_Line[i] - Right_Line[i - 3] <= -CORNER_CHANGE_3_THRESHOLD &&
           Right_Line[i] - Right_Line[i - 4] <= -CORNER_CHANGE_4_THRESHOLD)
        {
            Right_Down_Find = i;
        }

        if(Left_Down_Find != 0 && Right_Down_Find != 0)
        {
            break;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     寻找上拐点,十字使用
  @param     搜索范围的起点行,终点行
  @return    修改后的全局拐点
             Left_Up_Find=0;
             Right_Up_Find=0;
  Sample     Find_Up_Point(int start,int end)
  @note      使用之前先查看对应的变量与注释,没找到时对应的变量置0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Up_Point(int start, int end)
{
    int i, t;

    Left_Up_Find = 0;
    Right_Up_Find = 0;

    if(start < end)
    {
        t = start;
        start = end;
        end = t;
    }

    if(end < IMG_ROW - Search_Stop_Line)
    {
        end = IMG_ROW - Search_Stop_Line;
    }

    // 这里需要访问 i-3,所以最小不能小于3
    if(end < 3)
    {
        end = 3;
    }

    // 这里需要访问 i+4,所以最大不能超过 IMG_ROW-5
    if(start > IMG_ROW - 5)
    {
        start = IMG_ROW - 5;
    }

    for(i = start; i >= end; i--)
    {
        if(Left_Up_Find == 0 &&
           abs(Left_Line[i] - Left_Line[i - 1]) <= CORNER_STABLE_THRESHOLD &&
           abs(Left_Line[i - 1] - Left_Line[i - 2]) <= CORNER_STABLE_THRESHOLD &&
           abs(Left_Line[i - 2] - Left_Line[i - 3]) <= CORNER_STABLE_THRESHOLD &&
           Left_Line[i] - Left_Line[i + 2] >= CORNER_CHANGE_2_THRESHOLD &&
           Left_Line[i] - Left_Line[i + 3] >= CORNER_CHANGE_3_THRESHOLD &&
           Left_Line[i] - Left_Line[i + 4] >= CORNER_CHANGE_4_THRESHOLD)
        {
            Left_Up_Find = i;
        }

        if(Right_Up_Find == 0 &&
           abs(Right_Line[i] - Right_Line[i - 1]) <= CORNER_STABLE_THRESHOLD &&
           abs(Right_Line[i - 1] - Right_Line[i - 2]) <= CORNER_STABLE_THRESHOLD &&
           abs(Right_Line[i - 2] - Right_Line[i - 3]) <= CORNER_STABLE_THRESHOLD &&
           Right_Line[i] - Right_Line[i + 2] <= -CORNER_CHANGE_2_THRESHOLD &&
           Right_Line[i] - Right_Line[i + 3] <= -CORNER_CHANGE_3_THRESHOLD &&
           Right_Line[i] - Right_Line[i + 4] <= -CORNER_CHANGE_4_THRESHOLD)
        {
            Right_Up_Find = i;
        }

        if(Left_Up_Find != 0 && Right_Up_Find != 0)
        {
            break;
        }
    }

    /*
     * 最后确认左右拐点都找到了
     * 原来一个为0时也认为是真值,会导致误判
     */
    if(Left_Up_Find != 0 && Right_Up_Find != 0)
    {
        if(abs(Right_Up_Find - Left_Up_Find) > CORNER_UP_MAX_ROW_DIFF)
        {
            Right_Up_Find = 0;
            Left_Up_Find = 0;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左边界延长
  @param     延长起始行,延长到某行
  @return    null
  Sample     Lengthen_Left_Boundry(int start,int end);
  @note      从起始行往下找5个点求斜率,然后延长直到边界
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Left_Boundry(int start, int end)
{
    int i, t;
    float k = 0.0f;

    // 起始点位置校正,防止出现越界的情况
    if (start >= IMG_ROW - 1)
    {
        start = IMG_ROW - 1;
    }
    else if (start <= 0)
    {
        start = 0;
    }

    if (end >= IMG_ROW - 1)
    {
        end = IMG_ROW - 1;
    }
    else if (end <= 0)
    {
        end = 0;
    }

    // 这里使用 i++ 递增,如果起始点在下方就交换顺序
    if (end < start)
    {
        t = end;
        end = start;
        start = t;
    }

    /*
     * 需要读取 Left_Line[start - 4],
     * 起始行靠下的时候可能借不到延长斜率,只做直线延长
     */
    /* ★ 标记这几行"补过了" ★
       ★★ 这个函数是状态3 补线的主力, 少了这一句补线就完全没用 ★★
       下面的 else 分支是【直接写 Left_Line[i]】的, 不走 Left_Add_Line,
       所以不会自动打标记 —— 而丢线行的 Left_Lost_Flag 仍然是 1,
       Calculate_Mid_Line 的情况2b 和 TFT180_Show_Track_Lines 都会把这几行
       当成"没找到"直接忽略。结果就是: 补了等于没补, 屏幕上也看不见。
       (上面 start<=5 那条分支走 Left_Add_Line, 本来就带标记, 这里再打一次无害) */
    Isl_Patch_Mark(start, end);

    if (start <= 5)
    {
        Left_Add_Line(
            Left_Line[start],
            start,
            Left_Line[end],
            end
        );
    }
    else
    {
        /*
         * start 到 start-4 共4行,
         * k 表示每往下一行坐标变化多少
         */
        k = (float)(Left_Line[start] - Left_Line[start - 4]) / 4.0f;

        for (i = start; i <= end; i++)
        {
            /*
             * x = (y-y1) * k + x1
             */
            Left_Line[i] =
                (int)((i - start) * k + Left_Line[start]);

            if (Left_Line[i] >= IMG_COL - 1)
            {
                Left_Line[i] = IMG_COL - 1;
            }
            else if (Left_Line[i] <= 0)
            {
                Left_Line[i] = 0;
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右边界延长
  @param     延长起始行,延长到某行
  @return    null
  Sample     Lengthen_Right_Boundry(int start,int end);
  @note      从起始行往下找5个点求斜率,然后延长直到边界
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Right_Boundry(int start, int end)
{
    int i, t;
    float k = 0.0f;

    // 起始点位置校正,防止出现越界的情况
    if (start >= IMG_ROW - 1)
    {
        start = IMG_ROW - 1;
    }
    else if (start <= 0)
    {
        start = 0;
    }

    if (end >= IMG_ROW - 1)
    {
        end = IMG_ROW - 1;
    }
    else if (end <= 0)
    {
        end = 0;
    }

    // 这里使用 i++ 递增,如果起始点在下方就交换顺序
    if (end < start)
    {
        t = end;
        end = start;
        start = t;
    }

    /*
     * 需要读取 Right_Line[start - 4],
     * 起始行靠下的时候可能借不到延长斜率,只做直线延长
     */
    /* ★ 标记这几行"补过了" ★
       同 Lengthen_Left_Boundry: else 分支直接写 Right_Line[i], 不会自动打标记,
       不打的话 Calculate_Mid_Line 和显示层都会忽略, 补线白补。 */
    Isl_Patch_Mark(start, end);

    if (start <= 5)
    {
        Right_Add_Line(
            Right_Line[start],
            start,
            Right_Line[end],
            end
        );
    }
    else
    {
        /*
         * start 到 start-4 共4行,
         * k 表示每往下一行坐标变化多少
         */
        k = (float)(Right_Line[start] - Right_Line[start - 4]) / 4.0f;

        for (i = start; i <= end; i++)
        {
            /*
             * x = (y-y1) * k + x1
             */
            Right_Line[i] =
                (int)((i - start) * k + Right_Line[start]);

            if (Right_Line[i] >= IMG_COL - 1)
            {
                Right_Line[i] = IMG_COL - 1;
            }
            else if (Right_Line[i] <= 0)
            {
                Right_Line[i] = 0;
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     十字检测
  @param     null
  @return    null
  Sample     Cross_Detect(void);
  @note      检测四个拐点有没有都找到,四个点都找到才认为找到了十字
-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------
  @brief     十字补线结果的一阶低通
  @note      为什么要加这个:
            角点是逐帧搜出来的, 位置本来就会跳 1~3 个像素; 更麻烦的是下面补线那四个分支
            ("四个点都找到" / "只找到两个上点" / 只找到一边) 会来回切,
            补出来的形状一会儿是斜线、一会儿是垂直延长, 中线跟着跳,
            经 PID 放大之后就是前轮不停地左右摆。

            这里对补线后的左右边界做一阶低通。
            只在"上一帧也是十字"的时候才滤, 所以刚进十字那一帧用的是原始值,
            不会因为滤波而漏掉十字; 十字一结束也立刻停止滤波, 不会有拖尾。

            想关掉就把 CROSS_LINE_FILTER 改成 1.0f。
-------------------------------------------------------------------------------------------------------------------*/
#define CROSS_LINE_FILTER   0.35f   /* 越小越稳但越迟钝, 1.0 = 完全不滤波 */

static uint8 cross_flt_prev[IMG_ROW][2];    /* [行][0=左 1=右] 上一帧的边界 */
static uint8 cross_flt_on = 0;              /* 上一帧是不是十字 */

static void cross_line_filter(void)
{
    int i;

    /* ★ 刷屏优化: 不在十字里、上一帧也不在十字里, 这一趟循环什么都不会改
       (if (cross_flt_on) 不成立, prev 又只是原样快照), 直接返回。
       不会因此让 prev 变"陈旧": prev 只在 cross_flt_on==1 时被用到,
       而 cross_flt_on 只在 Cross_Flag==1 的那一帧才会被置 1 —— 那一帧必定会跑这个循环。 */
    if (!cross_flt_on && !Cross_Flag)
    {
        return;
    }

    for (i = 0; i < IMG_ROW; i ++)
    {
        int l = Left_Line[i];
        int r = Right_Line[i];

        if (cross_flt_on)
        {
            l = (int)((float)l * CROSS_LINE_FILTER +
                      (float)cross_flt_prev[i][0] * (1.0f - CROSS_LINE_FILTER) + 0.5f);
            r = (int)((float)r * CROSS_LINE_FILTER +
                      (float)cross_flt_prev[i][1] * (1.0f - CROSS_LINE_FILTER) + 0.5f);

            Left_Line[i]  = l;
            Right_Line[i] = r;
        }

        cross_flt_prev[i][0] = (uint8)l;
        cross_flt_prev[i][1] = (uint8)r;
    }

    cross_flt_on = (uint8)Cross_Flag;       /* 给下一帧用 */
}

void Cross_Detect(void)
{
    int down_search_start = 0; // 下拐点搜索起点

    Cross_Flag = 0;

    if (1) // 与环岛无关
    {
        Left_Up_Find  = 0;
        Right_Up_Find = 0;

        if (Both_Lost_Time >= 10) // 十字必定是双线丢线
        {
            Find_Up_Point(IMG_ROW - 1, 0);

            if (Left_Up_Find == 0 &&
                Right_Up_Find == 0) // 没有同时找到两个上拐点,直接结束
            {
                return;
            }
        }

        if (Left_Up_Find != 0 &&
            Right_Up_Find != 0) // 找到了两个上拐点,认为是十字
        {
            Cross_Flag = 1;

            // 两个向上拐点坐标靠上的作为下拐点搜索起点
            down_search_start =
                Left_Up_Find > Right_Up_Find ?
                Left_Up_Find : Right_Up_Find;

            // 向上拐点减2作为下拐点的截止行
            Find_Down_Point(
                IMG_ROW - 5,
                down_search_start + 2
            );

            if (Left_Down_Find <= Left_Up_Find)
            {
                Left_Down_Find = 0; // 下点不可能比上点还靠上
            }

            if (Right_Down_Find <= Right_Up_Find)
            {
                Right_Down_Find = 0; // 下点不可能比上点还靠上
            }

            if (Left_Down_Find != 0 &&
                Right_Down_Find != 0)
            {
                // 四个点都在,补两条线,补完后继续
                Left_Add_Line(
                    Left_Line[Left_Up_Find],
                    Left_Up_Find,
                    Left_Line[Left_Down_Find],
                    Left_Down_Find
                );

                Right_Add_Line(
                    Right_Line[Right_Up_Find],
                    Right_Up_Find,
                    Right_Line[Right_Down_Find],
                    Right_Down_Find
                );
            }
            else if (Left_Down_Find == 0 &&
                     Right_Down_Find != 0)
            {
                // 左补线
                Lengthen_Left_Boundry(
                    Left_Up_Find - 1,
                    IMG_ROW - 1
                );

                Right_Add_Line(
                    Right_Line[Right_Up_Find],
                    Right_Up_Find,
                    Right_Line[Right_Down_Find],
                    Right_Down_Find
                );
            }
            else if (Left_Down_Find != 0 &&
                     Right_Down_Find == 0)
            {
                // 右补线
                Left_Add_Line(
                    Left_Line[Left_Up_Find],
                    Left_Up_Find,
                    Left_Line[Left_Down_Find],
                    Left_Down_Find
                );

                Lengthen_Right_Boundry(
                    Right_Up_Find - 1,
                    IMG_ROW - 1
                );
            }
            else if (Left_Down_Find == 0 &&
                     Right_Down_Find == 0)
            {
                // 只有两个上拐点
                Lengthen_Left_Boundry(
                    Left_Up_Find - 1,
                    IMG_ROW - 1
                );

                Lengthen_Right_Boundry(
                    Right_Up_Find - 1,
                    IMG_ROW - 1
                );
            }
        }
        else
        {
            Cross_Flag = 0;
        }
    }

    /* 补线结果低通: 抑制十字里"补线分支来回切"造成的舵机抖动 */
    cross_line_filter();

    // 记得关闭变量,debug使用
//    ips200_showuint8(0, 12, Cross_Flag);
//    ips200_showuint8(0, 13, Island_State);
//    ips200_showuint8(50, 12, Left_Up_Find);
//    ips200_showuint8(100, 12, Right_Up_Find);
//    ips200_showuint8(50, 13, Left_Down_Find);
//    ips200_showuint8(100, 13, Right_Down_Find);
}

#define ROAD_WIDTH_DEFAULT     ((float)IMG_COL * 0.50f)
/* 原来是写死的 6.0f, 在 60 宽时占 10%, 换成 94 宽后只占 6.4%, 门槛偏松。
   改成按 IMG_COL 比例, 保持和原来一样的相对含义 */
#define ROAD_WIDTH_MIN         ((float)IMG_COL * (6.0f / 60.0f))
#define ROAD_WIDTH_FILTER      0.20f
#define MID_LINE_FILTER        0.25f

/*-------------------------------------------------------------------------------------------------------------------
  @brief     计算左右边界后的中线
  @param     null
  @return    null
  @note      支持双边界有效,单边丢线,十字补线和双边丢线的情况
-------------------------------------------------------------------------------------------------------------------*/
void Calculate_Mid_Line(void)
{
    int i;
    int search_top;

    float current_width;
    float current_mid;

    uint8_t left_valid;
    uint8_t right_valid;
    uint8_t row_searched;

    /*
     * 对每一行的赛道宽度做估计
     * 单边丢线时用估计值补出另一侧
     * 所以不能只使用一个固定的宽度
     */
    static float road_width_estimate[IMG_ROW];

    /*
     * 沿用上一帧中线
     * 当前帧双边界全丢失时,暂时沿用上一帧的结果
     */
    static float last_mid_line[IMG_ROW];

    static uint8_t initialized = 0;
    static uint8_t width_seed_gen = 0;      /* 赛宽估计已经播种到第几代: 0=还没播 1=默认值 2=标定值 */

    /*
     * 第一次进来时初始化
     */
    if (initialized == 0)
    {
        for (i = 0; i < IMG_ROW; i++)
        {
            last_mid_line[i] = (IMG_COL - 1) * 0.5f;
            Mid_Line[i] = (IMG_COL - 1) / 2;
        }

        initialized = 1;
    }

    /*
     * 单边丢线时要用"另一边 + 半个赛宽"推中线(见下面情况3/情况4), 所以赛宽估计的
     * 初值很关键。
     *
     * 原来初值是个盲猜的固定值 ROAD_WIDTH_DEFAULT(=IMG_COL*0.50), 只能靠
     * ROAD_WIDTH_FILTER 一帧一帧慢慢收敛; 而只有"左右边界都有效"的行才会更新它,
     * 图像上端那些经常两侧同时丢线的行根本收敛不了, 永远用着那个盲猜值。
     *
     * 现在改成用菜单5 标定出来的 Base_Width[] 逐行播种:
     *   - 开机播一次
     *   - 之后每次菜单5 标定完成(Width_Calib_State 变成 2)再重播一次,
     *     这样标完立刻生效, 不用重启
     *   - Base_Width[i] == 0 表示那一行没标定出来(顶部几行本来就不参与搜索),
     *     这种情况保留原值不动
     *
     * 注意: 播种只给初值, 后面仍然照常做低通, 所以转弯时赛宽在图像里的
     *       真实变化还是能跟得上。
     */
    if (width_seed_gen == 0 ||
        (width_seed_gen == 1 && Width_Calib_State == 2))
    {
        width_seed_gen = (Width_Calib_State == 2) ? 2 : 1;

        for (i = 0; i < IMG_ROW; i++)
        {
            if (width_seed_gen == 2)
            {
                if (Base_Width[i] > 0)
                {
                    road_width_estimate[i] = (float)Base_Width[i];
                }
            }
            else
            {
                road_width_estimate[i] = ROAD_WIDTH_DEFAULT;
            }
        }
    }

    /*
     * 最长白列实际已经算过这个
     *
     * 搜索范围:
     * IMG_ROW - Search_Stop_Line 到 IMG_ROW - 1
     */
    search_top = IMG_ROW - Search_Stop_Line;

    if (search_top < 0)
    {
        search_top = 0;
    }
    else if (search_top >= IMG_ROW)
    {
        search_top = IMG_ROW - 1;
    }

    /*
     * 第一遍:
     * 使用左右边界都存在的行做宽度估计
     */
    for (i = IMG_ROW - 1; i >= search_top; i--)
    {
        left_valid =
            (Left_Lost_Flag[i] == 0);

        right_valid =
            (Right_Lost_Flag[i] == 0);

        if (left_valid && right_valid)
        {
            current_width =
                (float)(Right_Line[i] - Left_Line[i]);

            /*
             * 排除掉太小的错误路宽
             */
            if (current_width >= ROAD_WIDTH_MIN &&
                current_width <= IMG_COL - 1)
            {
                road_width_estimate[i] =
                    (1.0f - ROAD_WIDTH_FILTER) *
                    road_width_estimate[i] +
                    ROAD_WIDTH_FILTER *
                    current_width;
            }
        }
    }

    /*
     * 第二遍:
     * 根据左右边界状态计算中线
     */
    for (i = IMG_ROW - 1; i >= 0; i--)
    {
        row_searched = (i >= search_top);

        left_valid =
            row_searched &&
            (Left_Lost_Flag[i] == 0);

        right_valid =
            row_searched &&
            (Right_Lost_Flag[i] == 0);

        /*
         * 情况0: 环岛状态3 专用 —— 用"边界 - 标准赛宽"直接算中线
         *
         * 为什么单独拿出来:
         *   状态3 车已经贴到环岛口, 右边界的搜索终点正好落在环岛的外沿上,
         *   所以 Right_Line 就是环岛的左沿; 从它往左退半个赛宽, 就是环岛左侧
         *   那条通道的正中间 —— 正是车该走的位置。
         *   而这一步左边界的补线不可靠(经常是丢线, 或者刚补上去的), 所以不用它。
         *
         * 标准赛宽用 Base_Width[] (菜单5 标定并固化下来的那份), 逐行都有值。
         * 系统只有左环岛时那条镜像处理, 用 Left_Line + 半个赛宽。
         *
         * 这个分支优先级最高, 但只在 Island_State==3 且该侧边界有效、
         * 该行有标定值时才生效, 其它情况照常走下面的常规逻辑。
         */
        if ((Island_State == 3) &&
            (Right_Island_Flag == 1) && right_valid && (Base_Width[i] > 0))
        {
            current_mid =
                (float)Right_Line[i] -
                (float)Base_Width[i] * ISL_MID_FACTOR;
        }
        else if ((Island_State == 3) &&
                 (Left_Island_Flag == 1) && left_valid && (Base_Width[i] > 0))
        {
            current_mid =
                (float)Left_Line[i] +
                (float)Base_Width[i] * ISL_MID_FACTOR;
        }

        /*
         * 情况1:左右边界都存在
         *
         * 十字处直接取双线均可见的行
         */
        else if (left_valid && right_valid)
        {
            current_mid =
                (Left_Line[i] + Right_Line[i]) * 0.5f;
        }

        /*
         * 情况2:十字检测已经补过线
         *
         * Cross_Detect() 补线后,边线标志位一定是丢线点,
         * 所以十字状态下可直接检查补线后的边界值
         *
         * 排除默认边界:
         * Left_Line=0 或 Right_Line=IMG_COL-1
         */
        else if (Cross_Flag == 1 &&
                 Right_Line[i] > Left_Line[i] &&
                 !(Left_Line[i] == 0 &&
                   Right_Line[i] == IMG_COL - 1))
        {
            current_mid =
                (Left_Line[i] + Right_Line[i]) * 0.5f;
        }

        /*
         * 情况2b: 环岛状态机补过线
         *
         * 和上面的情况2 完全对称 —— 只是把 Cross_Flag 换成 Isl_Patched[i]。
         *
         * 为什么必须有这个分支:
         *   补线(Left_Add_Line/Right_Add_Line)只改 Left_Line[]/Right_Line[],
         *   不会改 Left_Lost_Flag[]。而上面算 left_valid 时看的就是 Lost_Flag,
         *   所以补过的那几行 left_valid 仍然是 0, 会一路掉到情况3/4/5 去 ——
         *   补线写进去的值根本没人读。十字靠 Cross_Flag 开了门, 环岛没有,
         *   所以环岛的补线在此之前一直是白补的。
         *
         *   Isl_Patched[i] 由 Isl_Patch_Mark() 在补线时置位,
         *   在 Longest_White_Column() 重建边界时清零。详见 island.c。
         */
        else if (Isl_Patched[i] &&
                 Right_Line[i] > Left_Line[i] &&
                 !(Left_Line[i] == 0 &&
                   Right_Line[i] == IMG_COL - 1))
        {
            current_mid =
                (Left_Line[i] + Right_Line[i]) * 0.5f;
        }

        /*
         * 情况3:左边界丢失,右边界存在
         *
         * 中线 = 右边界 - 赛道宽度的一半
         */
        else if (!left_valid && right_valid)
        {
            current_mid =
                Right_Line[i] -
                road_width_estimate[i] * 0.5f;
        }

        /*
         * 情况4:右边界丢失,左边界存在
         *
         * 中线 = 左边界 + 赛道宽度的一半
         */
        else if (left_valid && !right_valid)
        {
            current_mid =
                Left_Line[i] +
                road_width_estimate[i] * 0.5f;
        }

        /*
         * 情况5:左右边界同时丢失
         */
        else
        {
            /*
             * 当前帧上方的行都丢线时,暂时沿用上一帧的最后一个中线
             */
            if (i < IMG_ROW - 1)
            {
                current_mid = Mid_Line[i + 1];
            }
            else
            {
                /*
                 * 图像最上方也双线丢失时,沿用上一帧的中线
                 */
                current_mid = last_mid_line[i];
            }
        }

        /*
         * 防止中线超出图像边界
         */
        if (current_mid < 0.0f)
        {
            current_mid = 0.0f;
        }
        else if (current_mid > IMG_COL - 1)
        {
            current_mid = IMG_COL - 1;
        }

        /*
         * 中线边界做滤波时做抑抖
         *
         * 75%使用本帧结果,25%使用上一帧结果
         * 可以减慢速度,防止中线突变
         */
        current_mid =
            (1.0f - MID_LINE_FILTER) * current_mid +
            MID_LINE_FILTER * last_mid_line[i];

        Mid_Line[i] = (int)(current_mid + 0.5f);

        last_mid_line[i] = current_mid;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     在TFT180图像上显示左边界,右边界和中线
  @param     x           图像显示区域的左上角x坐标
  @param     y           图像显示区域的左上角y坐标
  @param     dis_width   图像显示宽度
  @param     dis_height  图像显示高度
  @return    null
  Sample     TFT180_Show_Track_Lines(0, 61, 60, 60);
  @note      放在图像显示函数之后调用,否则会被图像覆盖
-------------------------------------------------------------------------------------------------------------------*/
void TFT180_Show_Track_Lines(uint16 x, uint16 y, uint16 dis_width, uint16 dis_height)
{
    int i;
    int left_x, right_x, mid_x, screen_y;
    int last_left_x = 0;
    int last_right_x = 0;
    int last_mid_x = 0;
    int last_left_y = 0;
    int last_right_y = 0;
    int last_mid_y = 0;

    uint8 left_last_valid = 0;
    uint8 right_last_valid = 0;
    uint8 mid_last_valid = 0;

    uint8 left_valid;
    uint8 right_valid;
    uint8 mid_valid;

    if(IMG_COL <= 1 || IMG_ROW <= 1 || dis_width <= 1 || dis_height <= 1)
    {
        return;
    }

    for(i = 0; i < IMG_ROW; i++)
    {
        screen_y = y + (int)((uint32)i * (dis_height - 1) / (IMG_ROW - 1));

        /* Isl_Patched[i] 和 Cross_Flag 是一个道理:
           这一行的边界虽然 Lost_Flag 说是丢线, 但它是环岛状态机补出来的, 要画。
           不加这一项的话, 环岛的补线在屏幕上永远看不见 ——
           而补线补的恰恰就是丢线的那几行 */
        left_valid  = (Left_Lost_Flag[i]  == 0 || Cross_Flag == 1 || Isl_Patched[i]);
        right_valid = (Right_Lost_Flag[i] == 0 || Cross_Flag == 1 || Isl_Patched[i]);
        mid_valid = 1;

        if(Left_Line[i] < 0 || Left_Line[i] >= IMG_COL)
        {
            left_valid = 0;
        }

        if(Right_Line[i] < 0 || Right_Line[i] >= IMG_COL)
        {
            right_valid = 0;
        }

        if(Mid_Line[i] < 0 || Mid_Line[i] >= IMG_COL)
        {
            mid_valid = 0;
        }

        // 左边界:红色
        if(left_valid)
        {
            left_x = x + (int)((uint32)Left_Line[i] * (dis_width - 1) / (IMG_COL - 1));

            if(left_last_valid)
            {
                tft180_draw_line((uint16)last_left_x, (uint16)last_left_y, (uint16)left_x, (uint16)screen_y, RGB565_RED);
            }
            else
            {
                tft180_draw_point((uint16)left_x, (uint16)screen_y, RGB565_RED);
            }

            last_left_x = left_x;
            last_left_y = screen_y;
            left_last_valid = 1;
        }
        else
        {
            left_last_valid = 0;
        }

        // 右边界:绿色
        if(right_valid)
        {
            right_x = x + (int)((uint32)Right_Line[i] * (dis_width - 1) / (IMG_COL - 1));

            if(right_last_valid)
            {
                tft180_draw_line((uint16)last_right_x, (uint16)last_right_y, (uint16)right_x, (uint16)screen_y, RGB565_BLUE);
            }
            else
            {
                tft180_draw_point((uint16)right_x, (uint16)screen_y, RGB565_BLUE);
            }

            last_right_x = right_x;
            last_right_y = screen_y;
            right_last_valid = 1;
        }
        else
        {
            right_last_valid = 0;
        }

        // 中线:蓝色
        if(mid_valid)
        {
            mid_x = x + (int)((uint32)Mid_Line[i] * (dis_width - 1) / (IMG_COL - 1));

            if(mid_last_valid)
            {
                tft180_draw_line((uint16)last_mid_x, (uint16)last_mid_y, (uint16)mid_x, (uint16)screen_y, RGB565_GREEN);
            }
            else
            {
                tft180_draw_point((uint16)mid_x, (uint16)screen_y, RGB565_GREEN);
            }

            last_mid_x = mid_x;
            last_mid_y = screen_y;
            mid_last_valid = 1;
        }
        else
        {
            mid_last_valid = 0;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     斑马线检测
  @param     null
  @return    null
  Sample     Zebra_Stripes_Detect(void)
  @note      边界起始行往下,最长白列较长说明道路宽
-------------------------------------------------------------------------------------------------------------------*/
// void Zebra_Stripes_Detect(void)
// {
//     int i=0,j=0;
//     int change_count=0;//变化次数
//     int start_line=0;
//     int endl_ine=0;
//     int narrow_road_count=0;
//     if(Cross_Flag!=0||(1<=Ramp_Flag&&Ramp_Flag<=3)||Zebra_Stripes_Flag!=0||
//        Stop_Flag!=0 )//元素互斥,满足条件||Electromagnet_Flag!=0||Barricade_Flag!=0
//     {
//         return;
//     }

//     ////计算变化行并保存
//     if(Search_Stop_Line>=60&&
//        30<=Longest_White_Column_Left[1]&&Longest_White_Column_Left[1]<=IMG_COL-30&&
//        30<=Longest_White_Column_Right[1]&&Longest_White_Column_Right[1]<=IMG_COL-30&&
//        Boundry_Start_Left>=IMG_ROW-15&&Boundry_Start_Right>=IMG_ROW-15)
//     {//防止行冲突,最长白列的位置就是四个拐点边界起始点靠近
//         for(i=59;i>=20;i--)//在可行的行内寻找左边界宽度过窄的地方
//         {
//             if((Standard_Road_Wide[i]-Road_Wide[i])>10)
//             {
//                 narrow_road_count++;//连续过窄的行数
//                 if(narrow_road_count>=5)
//                 {
//                     start_line=i;//记录下连续过窄的位置
//                     break;
//                 }
//             }
//         }
//     }
//     if(start_line!=0)//连续过窄的行数过多,以起点设置一段范围补线
//     {
//         start_line=start_line+8;
//         endl_ine=start_line-15;
//         if(start_line>=IMG_ROW-1)//无法补线,防止越界
//         {
//             start_line=IMG_ROW-1;
//         }
//         if(endl_ine<=0)//无法补线,防止越界
//         {
//             endl_ine=0;
//         }
//         for(i=start_line;i>=endl_ine;i--)//开始补线
//         {
//             for(j=Left_Line[i];j<=Right_Line[i];j++)
//             {
//                 if(BinaryImg_CDM[i][j+1]-BinaryImg_CDM[i][j]!=0)
//                 {
//                     change_count++;

//                 }
//             }
//         }
// //        ips200_show_uint(0*16,100,change_count,5);//debug使用,查看变化次数
//     }
//  //补线方向,比赛有bug,debug使用
// //        Draw_Line( Left_Line[start_line], start_line, Left_Line[endl_ine], endl_ine);
// //        Draw_Line( Left_Line[start_line], start_line, Right_Line[start_line], start_line);
// //        Draw_Line(Right_Line[endl_ine], endl_ine, Right_Line[start_line], start_line);
// //        Draw_Line(Right_Line[endl_ine], endl_ine, Left_Line[endl_ine], endl_ine);
// //        ips200_draw_line ( Left_Line[start_line], start_line, Left_Line[endl_ine], endl_ine, RGB565_RED);
// //        ips200_draw_line ( Left_Line[start_line], start_line, Right_Line[start_line], start_line, RGB565_RED);
// //        ips200_draw_line (Right_Line[endl_ine], endl_ine, Right_Line[start_line], start_line, RGB565_RED);
// //        ips200_draw_line (Right_Line[endl_ine], endl_ine, Left_Line[endl_ine], endl_ine, RGB565_RED);

//     if(change_count>30)//变化超过某一阈值,认为找到了斑马线
//     {
//         Zebra_Stripes_Flag=1;
//     }
// //屏幕调试显示,debug使用
// //    ips200_show_uint(0*16,50,narrow_road_count,5);
// //    ips200_show_uint(1*16,50,change_count,5);
// }


/* ==========================================================================================
 *                                      斑马线检测 + 任务状态机
 *
 *  比赛要求: 车起始停在斑马线后面 -> 按发车键启动 -> 越过斑马线后自动停车
 *
 *  流程:  IDLE(还没看到) --看到--> CROSSING(正在通过) --看不到了--> PASSED(已越过)
 *        --再走 ZEBRA_EXTRA_DIST--> DONE(停车)
 * ========================================================================================== */

/* ---- 单帧检测参数 ---- */
#define ZEBRA_SCAN_ROWS     20      /* 在图像底部多少行/列的范围内扫描 */
#define ZEBRA_JUMP_MIN      6       /* 一行(列)里黑白跳变超过这个数, 才算斑马线的一行 */
#define ZEBRA_LINE_MIN      4       /* 至少这么多行(列)符合, 才认为本帧看到了斑马线 */

/* ---- 状态机参数 ---- */
#define ZEBRA_HIT_FRAMES    3       /* 连续这么多帧看到才确认(去抖) */
#define ZEBRA_LOST_FRAMES   8       /* 连续这么多帧看不到, 才开始怀疑已经开过去了 */

/* ★ 关键参数 1 ★
   从"最后一次看到斑马线"算起, 车必须真的再往前开够这么多编码器计数,
   才承认是"这一次开过去了"。

   为什么需要它: 车刚发车时加速、图像运动模糊, 很容易连续几帧检测不到斑马线。
   如果光凭"看不到"就判定越过, 一次经过会被误判成通过, 圈数就数乱了。
   加上这个距离门槛之后, 只有车真的往前走了才算数;
   中途只要重新看到斑马线, 计数立刻清零重来。

   标定方法: 屏幕菜单1 第 11 行的 D 值就是它(看不到时才累加)。
   单位是编码器计数, 约 1000 计数 ≈ 车轮转一圈 ≈ 0.2m */
#define ZEBRA_CONFIRM_DIST  2000

/* ★ 关键参数 2 ★
   第 1 次越过斑马线之后, 必须再跑够这么多编码器计数,
   第 2 次"看到斑马线"才作数。

   为什么需要它: 刚越过时车尾可能还在斑马线上, 或者检测抖动又认出一帧,
   如果不加这个间隔, 同一条斑马线会被数成"第 2 次", 车刚发车就停了。
   只要它明显小于赛道一圈的长度就行(一圈一般好几米, 4000 计数才 0.8m 左右)。
   标定方法: 屏幕菜单1 第 12 行的 R 值就是它(第 1 次越过后才累加) */
#define ZEBRA_REARM_DIST    4000

/* 第 2 次越过之后再走多少编码器计数才停车。0 = 确认越过就立刻停。
   车头刚过线时后轮还压在线上, 想停得靠后一点就加大这个值 */
#define ZEBRA_EXTRA_DIST    0

volatile uint8  Zebra_Task_State = ZEBRA_STATE_IDLE;
volatile uint8  Zebra_Seen_Flag  = 0;
volatile uint8  Zebra_Pass_Cnt   = 0;       /* 已经越过几次: 0/1/2 */
volatile uint32 Zebra_Lost_Dist  = 0;       /* 看不到斑马线之后累计走的距离, 供屏幕调试 */
volatile uint32 Zebra_Rearm_Dist = 0;       /* 第 1 次越过后累计走的距离, 供屏幕调试 */

static uint8  zebra_hit_cnt    = 0;     /* 连续看到斑马线的帧数 */
static uint8  zebra_lost_cnt   = 0;     /* 连续看不到斑马线的帧数 */
static uint8  zebra_pass_cnt   = 0;     /* 已经越过几次(Zebra_Pass_Cnt 的本体) */
static uint32 zebra_lost_dist  = 0;     /* 看不到斑马线之后累计走过的编码器计数 */
static uint32 zebra_rearm_dist = 0;     /* 第 1 次越过后累计走过的编码器计数 */
static uint32 zebra_pass_dist  = 0;     /* 第 2 次越过后累计走过的编码器计数 */

extern volatile int GO;                 /* 定义在 isr.c */

/*-------------------------------------------------------------------------------------------------------------------
  @brief     斑马线检测(单帧)
  @return    1 = 本帧在图像里看到了斑马线, 0 = 没有
  @note      原理: 正常赛道一行里只有左右两条边线, 也就是 2 个黑白跳变;
             斑马线是横跨赛道的一组黑白条纹, 扫过一行会出现很多次跳变。
             横向和纵向都扫一遍, 这样条纹是横着排还是竖着排都能认出来。

             用 BinaryImg(原始二值图)而不是 BinaryImg_CDM ——
             CDM 只保留连通的主道, 斑马线的条纹可能已经被切掉了。
-------------------------------------------------------------------------------------------------------------------*/
uint8 Zebra_Detect(void)
{
    int i, j;
    int h_line = 0;     /* 横向跳变达标的行数 */
    int v_line = 0;     /* 纵向跳变达标的列数 */

    /* ---- 横向扫描 ---- */
    for (i = IMG_ROW - 1; i > IMG_ROW - 1 - ZEBRA_SCAN_ROWS; i--)
    {
        int jump = 0;

        if (i < 0)
        {
            break;
        }

        for (j = 1; j < IMG_COL; j++)
        {
            if (BinaryImg[i][j] != BinaryImg[i][j - 1])
            {
                jump++;
            }
        }

        if (jump >= ZEBRA_JUMP_MIN)
        {
            h_line++;
        }
    }

    /* ---- 纵向扫描(只扫中间 1/3 的列, 避开两侧边线) ---- */
    for (j = IMG_COL / 3; j < IMG_COL * 2 / 3; j++)
    {
        int jump = 0;

        for (i = IMG_ROW - 1; i > IMG_ROW - 1 - ZEBRA_SCAN_ROWS; i--)
        {
            if (i <= 0)
            {
                break;
            }

            if (BinaryImg[i][j] != BinaryImg[i - 1][j])
            {
                jump++;
            }
        }

        if (jump >= ZEBRA_JUMP_MIN)
        {
            v_line++;
        }
    }

    return ((h_line >= ZEBRA_LINE_MIN) || (v_line >= ZEBRA_LINE_MIN)) ? 1 : 0;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     斑马线任务复位
  @note      每次发车(GO 由 0 变 1)时调用, 让状态机从头开始
-------------------------------------------------------------------------------------------------------------------*/
void Zebra_Task_Reset(void)
{
    Zebra_Task_State = ZEBRA_STATE_IDLE;
    Zebra_Seen_Flag  = 0;
    Zebra_Pass_Cnt   = 0;
    Zebra_Lost_Dist  = 0;
    Zebra_Rearm_Dist = 0;
    zebra_hit_cnt    = 0;
    zebra_lost_cnt   = 0;
    zebra_pass_cnt   = 0;
    zebra_lost_dist  = 0;
    zebra_rearm_dist = 0;
    zebra_pass_dist  = 0;
}

/* ==========================================================================================
 *  ★ 丢图保护 (冲出赛道自动停车) ★
 *
 *  为什么不用"最长白列为 0"当判据:
 *      那个量是从图像最底行往上数连续白点的个数, 车压在斑马线黑色条纹上的时候
 *      底部整行都是黑的, 最长白列同样是 0 —— 和冲出赛道完全没法区分, 会误停车。
 *      而且如果冲到浅色地面上, 图像全白, 最长白列是最大值 60, 又抓不到。
 *
 *  改用"整幅二值图的白点总数" (Img_White_Cnt, 在 GetBinarizedImage 里顺带统计, 不花额外时间):
 *      赛道是白的, 背景是暗的, 所以
 *          正常在赛道上跑      -> 几千个白点
 *          压斑马线黑条纹      -> 白点还是很多(条纹只占一小部分), 不会误判
 *          冲出赛道/摄像头被挡 -> 白点很少            -> 判据 A
 *          冲到浅色地面        -> 白点几乎拉满        -> 判据 B (再叠加"边线全丢")
 *
 *  连续 IMG_LOST_FRAMES 帧都成立才真的停车, 避免单帧噪点把车停死。
 *  停车用的是和斑马线一样的开关 GO = -1, TIM6 里会自动 wheel_ctrl(0,0)。
 * ========================================================================================== */

#define IMG_LOST_FRAMES     8       /* 判据要连续成立多少帧才停车。8帧 @50fps ≈ 160ms */
#define IMG_LOST_WHITE_MIN  1200    /* 判据A: 白点少于这个数 -> 整幅图基本是黑的(共 5640 点) */
#define IMG_LOST_WHITE_MAX  5400    /* 判据B: 白点多于这个数 -> 整幅图几乎全白(共 5640 点) */
#define IMG_LOST_NOLINE     50      /* 判据B 的附加条件: 左右边线丢线行数都要超过这个 */

volatile int Img_Lost_Cnt  = 0;     /* 判据连续成立的帧数, 菜单3 显示用 */
volatile int Img_Lost_Flag = 0;     /* 1 = 这一轮跑动中因为丢图停过车 */
volatile int Img_Lost_On   = 1;     /* 1 = 开保护, 0 = 关。菜单4 可以切换 */

/*-------------------------------------------------------------------------------------------------------------------
  @brief     丢图保护, 每个摄像头帧调用一次
  @note      判据任意一条成立即记为"本帧丢图", 连续 IMG_LOST_FRAMES 帧才停车
-------------------------------------------------------------------------------------------------------------------*/
void Img_Lost_Tick(void)
{
    static uint8 was_running = 0;
    uint8 lost = 0;

    /* ---- 不在跑就不管(停车状态下列的数值没有意义) ---- */
    if ((GO != 1) || (Img_Lost_On == 0))
    {
        Img_Lost_Cnt = 0;
        was_running  = 0;
        return;
    }

    /* 刚起步: 把上一轮的丢图标记清掉, 这样菜单上看到的一定是这一轮的 */
    if (was_running == 0)
    {
        was_running   = 1;
        Img_Lost_Flag = 0;
    }

    /* ---- 判据 A: 整幅图基本是黑的 (冲出赛道 / 摄像头被挡) ---- */
    if (Img_White_Cnt < IMG_LOST_WHITE_MIN)
    {
        lost = 1;
    }
    /* ---- 判据 B: 整幅图几乎全白, 而且左右边线一根都找不到 (冲到浅色地面) ---- */
    else if ((Img_White_Cnt > IMG_LOST_WHITE_MAX) &&
             (Left_Lost_Time  >= IMG_LOST_NOLINE) &&
             (Right_Lost_Time >= IMG_LOST_NOLINE))
    {
        lost = 1;
    }

    if (lost)
    {
        Img_Lost_Cnt ++;

        if (Img_Lost_Cnt >= IMG_LOST_FRAMES)
        {
            GO = -1;                        /* 和斑马线停车共用同一个开关 */
            Img_Lost_Flag = 1;
            Img_Lost_Cnt  = IMG_LOST_FRAMES;    /* 停在阈值上, 菜单看得见 */
        }
    }
    else
    {
        Img_Lost_Cnt = 0;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     斑马线任务主循环, 每个摄像头帧调用一次(放在图像处理之后)
  @note      ★ 这里同时兼作"每帧任务"的入口 ★
             main.c 是 GBK 编码, 我这边改会把它中文注释搞乱, 所以没有在 main 里单独加行,
             而是借 Zebra_Task_Tick 这个每帧必到的位置调用下面几个任务。
             它们都依赖 Longest_White_Column() 的结果, 放在这里时序正好
             (main 的顺序是: ... → Zebra_Task_Tick() → Width_Calib_Tick())。
             你要是愿意自己动 main.c, 把这两行搬到 Zebra_Task_Tick() 前面效果完全一样。
-------------------------------------------------------------------------------------------------------------------*/
void Zebra_Task_Tick(void)
{
    Speed_Schedule_Update();    /* 按视野长度自动算目标速度(直道快、弯道慢) */
    Img_Lost_Tick();            /* 丢图保护(冲出赛道自动停车) */

    Zebra_Seen_Flag     = Zebra_Detect();
    Zebra_Stripes_Flag  = Zebra_Seen_Flag;      /* 供屏幕显示/调试用 */

    switch (Zebra_Task_State)
    {
        /* ---- 还没看到斑马线 ---- */
        case ZEBRA_STATE_IDLE:
            if (Zebra_Seen_Flag)
            {
                if (++zebra_hit_cnt >= ZEBRA_HIT_FRAMES)
                {
                    Zebra_Task_State = ZEBRA_STATE_CROSSING;
                    zebra_lost_cnt   = 0;
                    zebra_lost_dist  = 0;
                    Zebra_Lost_Dist  = 0;
                }
            }
            else
            {
                zebra_hit_cnt = 0;
            }
            break;

        /* ---- 已经看到, 正在通过(第1次或第2次) ---- */
        case ZEBRA_STATE_CROSSING:
            if (!Zebra_Seen_Flag)
            {
                /* 只有车在跑的时候才统计"看不到了" ——
                   车停着不动时视野不会变, 统计了会误判 */
                if (GO == 1)
                {
                    zebra_lost_cnt++;
                    Zebra_Lost_Dist = zebra_lost_dist;

                    /* 两个条件都满足才算"这一次真的开过去了":
                       1) 连续若干帧看不到(去抖)
                       2) 从最后一次看到算起, 车真的往前开够了 ZEBRA_CONFIRM_DIST
                      第 2 条专门用来对付运动模糊导致的连续几帧检测不到 */
                    if ((zebra_lost_cnt >= ZEBRA_LOST_FRAMES) &&
                        (zebra_lost_dist >= ZEBRA_CONFIRM_DIST))
                    {
                        zebra_hit_cnt = 0;

                        if (zebra_pass_cnt == 0)
                        {
                            /* === 第 1 次越过 ===
                               就是发车时看到的那条斑马线。不停车,
                               转入绕圈状态, 等下一次再看到斑马线 */
                            zebra_pass_cnt   = 1;
                            Zebra_Pass_Cnt   = 1;
                            zebra_rearm_dist = 0;
                            Zebra_Rearm_Dist = 0;
                            Zebra_Task_State = ZEBRA_STATE_REARM;
                        }
                        else
                        {
                            /* === 第 2 次越过 ===
                               绕完一圈又回到斑马线, 这次要停车了 */
                            zebra_pass_cnt   = 2;
                            Zebra_Pass_Cnt   = 2;
                            zebra_pass_dist  = 0;
                            Zebra_Task_State = ZEBRA_STATE_PASSED;
                        }
                    }
                }
            }
            else
            {
                /* 又看到了 —— 说明刚才只是抖动, 计数全部清零重来 */
                zebra_lost_cnt  = 0;
                zebra_lost_dist = 0;
                Zebra_Lost_Dist = 0;
            }
            break;

        /* ---- 第 1 次已越过, 绕圈中, 等再次看到斑马线 ---- */
        case ZEBRA_STATE_REARM:
            /* 刚越过的这一小段路里, 检测抖动可能又"看到"斑马线,
               所以必须先跑够 ZEBRA_REARM_DIST, 之后的看到才算数 */
            if (zebra_rearm_dist < ZEBRA_REARM_DIST)
            {
                zebra_hit_cnt = 0;
                break;
            }

            if (Zebra_Seen_Flag)
            {
                if (++zebra_hit_cnt >= ZEBRA_HIT_FRAMES)
                {
                    /* 绕了一圈又看到斑马线, 开始第 2 次通过 */
                    zebra_lost_cnt   = 0;
                    zebra_lost_dist  = 0;
                    Zebra_Lost_Dist  = 0;
                    Zebra_Task_State = ZEBRA_STATE_CROSSING;
                }
            }
            else
            {
                zebra_hit_cnt = 0;
            }
            break;

        /* ---- 第 2 次已越过, 再走一小段就停 ---- */
        case ZEBRA_STATE_PASSED:
#if (ZEBRA_EXTRA_DIST > 0)
            if (zebra_pass_dist < ZEBRA_EXTRA_DIST)
            {
                break;                          /* 还没走够, 继续等 */
            }
#endif
            Zebra_Task_State = ZEBRA_STATE_DONE;
            GO = -1;                            /* 绕完一圈再次越过斑马线, 停车 */
            break;

        default:
            break;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     斑马线任务里程累计
  @param     speed_l / speed_r  左右轮当前速度(编码器计数/10ms)
  @note      由 TIM6 中断每 10ms 调用一次。用编码器里程而不是定时,
             这样换了目标速度也不用重新标定 ZEBRA_EXTRA_DIST。
-------------------------------------------------------------------------------------------------------------------*/
void Zebra_Task_Distance(int speed_l, int speed_r)
{
    int s = (speed_l + speed_r) / 2;

    if (s < 0)
    {
        s = -s;
    }

    switch (Zebra_Task_State)
    {
        case ZEBRA_STATE_CROSSING:
            /* 本帧看不到斑马线 —— 累计距离, 用来判断是真的开过去了还是只是抖动。
               下一帧只要重新看到, Zebra_Task_Tick() 会把它清零 */
            if (Zebra_Seen_Flag == 0)
            {
                zebra_lost_dist += (uint32)s;
                Zebra_Lost_Dist  = zebra_lost_dist;
            }
            break;

        case ZEBRA_STATE_REARM:
            /* 第 1 次越过后一直在跑圈, 一直累加(不管看不看得到) */
            zebra_rearm_dist += (uint32)s;
            Zebra_Rearm_Dist  = zebra_rearm_dist;
            break;

        case ZEBRA_STATE_PASSED:
            /* 第 2 次已确认越过, 累计"再多走一段"的里程 */
            zebra_pass_dist += (uint32)s;
            break;

        default:
            break;
    }
}


/* ==========================================================================================
 *                        直道标准赛宽标定  (菜单 5)
 *
 *  原理
 *      直道上赛道宽度处处相等(比如固定 45cm), 所以图像里第 i 行的"赛道像素宽度"
 *      只由透视关系决定 —— 只跟行号 i 有关, 和车在直道上的前后位置、左右位置都无关
 *      (前后移动不改变透视, 左右偏移只是让左右边界一起平移, 差值不变)。
 *      唯一要求是车头方向平行于赛道边线, 也就是车要摆正。
 *
 *  做法
 *      车在直道上摆正、静止  ->  触发采集  ->  连续采 WIDTH_CALIB_FRAMES 帧
 *      逐行累加 (Right_Line[i] - Left_Line[i]), 采完求平均
 *
 *      为什么要多帧平均: 单帧边界是搜出来的, 本身有 1~2 像素抖动, 平均之后才稳。
 *      为什么要剔除丢线行: 丢线时是 Left_Line=0 / Right_Line=IMG_COL-1,
 *                          算出来宽度是 93 的假值, 混进去会把平均值带偏。
 * ========================================================================================== */

#define WIDTH_CALIB_FRAMES  32      /* 采集帧数。32 帧约 0.5 秒; 注意 width_sum 是 uint16,
                                       93 * 帧数 不要超过 65535, 也就是帧数别超过 700 */
#define WIDTH_CALIB_MINHIT  16      /* 一行至少要有这么多帧有效(一半), 才算标定成功 */

uint8    Base_Width[IMG_ROW];           /* 标定结果: 每行的标准赛宽(像素)。0 = 该行无效 */
uint8    Base_Width_Ok[IMG_ROW];        /* 该行是否标定成功 */
uint8    Base_Width_Range[IMG_ROW];     /* 该行采集期间的最大波动(像素), 越小越可信 */
uint8    Base_Width_Bad     = 0;        /* 无效行的数量 */
uint8    Base_Width_MaxRng  = 0;        /* 所有有效行里最大的波动值 */
volatile uint8 Width_Calib_State = 0;   /* 0 空闲  1 采集中  2 采集完成 */
volatile uint8 Width_Calib_Prog  = 0;   /* 采集进度 0 ~ WIDTH_CALIB_FRAMES */

static uint16 width_sum[IMG_ROW];
static uint8  width_min[IMG_ROW];
static uint8  width_max[IMG_ROW];
static uint8  width_hit[IMG_ROW];

/*-------------------------------------------------------------------------------------------------------------------
  ★ 固化的标准赛宽 ★
  这是菜单5 实测标定出来的一份结果, 直接写死在这里当开机初值, 这样每次上电不用重新标。

  来源: 车摆在直道上采集 32 帧平均 (OK:55/60  RNG:2)
        - 第 0~4 行是 0: 这五行在 IMG_TOP(=5) 以上, 本来就不参与搜索, 标不出来是正常的
        - 第 59 行约 72 像素

  怎么用:
    开机时 Width_Calib_LoadDefault() 会把这份数据拷进 Base_Width[],
    效果和"刚标定完"完全一样 —— 中线计算会拿它当单边丢线时的赛宽,
    菜单5 也会直接显示这份数据(而不是一片 0)。
    之后想重新标定, 进菜单5 按 KEY2 照常采集, 采完会覆盖掉。

  要换车/换摄像头/改俯仰角, 就重新标一次, 然后把菜单5 显示的 60 个数抄到这里替换。
  ★ 改这里之后记得 8 个一行, 一共 60 个数, 别多别少 ★
-------------------------------------------------------------------------------------------------------------------*/
const uint8 Base_Width_Default[IMG_ROW] =
{
    /* row  0 -  7 */    0,  0,  0,  0,  0,  6,  7, 10,
    /* row  8 - 15 */   11, 12, 13, 14, 15, 16, 17, 20,
    /* row 16 - 23 */   22, 23, 24, 25, 26, 27, 28, 30,
    /* row 24 - 31 */   32, 33, 34, 35, 36, 37, 38, 40,
    /* row 32 - 39 */   42, 43, 44, 45, 46, 47, 48, 49,
    /* row 40 - 47 */   50, 52, 53, 54, 55, 56, 58, 59,
    /* row 48 - 55 */   60, 61, 62, 63, 64, 65, 66, 67,
    /* row 56 - 59 */   68, 69, 71, 72
};

/*-------------------------------------------------------------------------------------------------------------------
  @brief    把固化的标准赛宽装进 Base_Width[], 开机时调用一次
  @note     装完之后状态和"刚标定完"一样, 菜单5 会直接显示这份数据
-------------------------------------------------------------------------------------------------------------------*/
void Width_Calib_LoadDefault(void)
{
    int i;

    for (i = 0; i < IMG_ROW; i ++)
    {
        Base_Width[i]       = Base_Width_Default[i];
        Base_Width_Ok[i]    = (Base_Width_Default[i] > 0) ? 1 : 0;
        Base_Width_Range[i] = 0;
    }

    Base_Width_Bad    = 0;
    Base_Width_MaxRng = 0;

    for (i = 0; i < IMG_ROW; i ++)
    {
        if (Base_Width_Default[i] == 0)
        {
            Base_Width_Bad ++;
        }
    }

    Width_Calib_Prog  = WIDTH_CALIB_FRAMES;
    Width_Calib_State = 2;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     开始一次赛宽标定, 把上一次的结果和统计量全部清零
-------------------------------------------------------------------------------------------------------------------*/
void Width_Calib_Start(void)
{
    int i;

    for (i = 0; i < IMG_ROW; i ++)
    {
        width_sum[i] = 0;
        width_min[i] = 255;
        width_max[i] = 0;
        width_hit[i] = 0;

        Base_Width[i]       = 0;
        Base_Width_Ok[i]    = 0;
        Base_Width_Range[i] = 0;
    }

    Base_Width_Bad    = 0;
    Base_Width_MaxRng = 0;
    Width_Calib_Prog  = 0;
    Width_Calib_State = 1;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     放弃 / 清空标定结果
-------------------------------------------------------------------------------------------------------------------*/
void Width_Calib_Clear(void)
{
    /* 不做"清零", 而是恢复成固化的那份初值 ——
       清零会让中线计算退回到盲猜的 ROAD_WIDTH_DEFAULT, 那比固化值差得多 */
    Width_Calib_LoadDefault();
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     赛宽标定的一帧, 在主循环里每处理完一帧图像调用一次
  @note      必须放在 Longest_White_Column() 之后, 这样读到的是当前帧的左右边界
-------------------------------------------------------------------------------------------------------------------*/
void Width_Calib_Tick(void)
{
    int i;

    if (Width_Calib_State != 1)
    {
        return;
    }

    /* ---- 逐行累加本帧的宽度 ---- */
    for (i = 0; i < IMG_ROW; i ++)
    {
        /* 只统计"左右边界都真的搜到了"的行 */
        if ((Left_Line[i] > 0) && (Right_Line[i] < IMG_COL - 1) &&
            (Right_Line[i] > Left_Line[i]))
        {
            uint8 w = (uint8)(Right_Line[i] - Left_Line[i]);

            width_sum[i] += w;
            width_hit[i] ++;

            if (w < width_min[i]) { width_min[i] = w; }
            if (w > width_max[i]) { width_max[i] = w; }
        }
    }

    Width_Calib_Prog ++;

    if (Width_Calib_Prog < WIDTH_CALIB_FRAMES)
    {
        return;                             /* 还没采够, 继续 */
    }

    /* ---- 采集结束, 逐行求平均 ---- */
    Base_Width_Bad    = 0;
    Base_Width_MaxRng = 0;

    for (i = 0; i < IMG_ROW; i ++)
    {
        if (width_hit[i] >= WIDTH_CALIB_MINHIT)
        {
            Base_Width[i]       = (uint8)(width_sum[i] / width_hit[i]);
            Base_Width_Ok[i]    = 1;
            Base_Width_Range[i] = (uint8)(width_max[i] - width_min[i]);

            if (Base_Width_Range[i] > Base_Width_MaxRng)
            {
                Base_Width_MaxRng = Base_Width_Range[i];
            }
        }
        else
        {
            /* 这一行大部分帧都是丢线的, 标不出来, 留 0 让使用者知道 */
            Base_Width[i]       = 0;
            Base_Width_Ok[i]    = 0;
            Base_Width_Range[i] = 0;
            Base_Width_Bad ++;
        }
    }

    Width_Calib_State = 2;
}