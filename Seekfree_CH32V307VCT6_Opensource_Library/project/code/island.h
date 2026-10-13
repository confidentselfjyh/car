#ifndef ISLAND_H
#define ISLAND_H

#include "zf_common_headfile.h"
#include "Ourcode_headfile.h"

/* ==========================================================================================
 *  环岛识别 (移植自开源工程 Island.c, 原作者图 188x120, 本工程 94x60)
 *
 *  换算原则: 两个方向都正好是 1/2, 所以
 *      行号 / 列号 / 行数 / 列数  ->  原值 / 2
 *      角度(60 / 200 / 320)      ->  不缩放 !
 *
 *  所有换算后的阈值都集中在 island.c 顶部的宏里, 实车调参改那里就行。
 * ========================================================================================== */

/* ==========================================================================================
 *  ★ 状态3 的转弯方案 (两个, 默认只开第二个) ★
 *
 *  方案一: 半圆补线 (ISL_ARC_ENABLE)
 *      以角点为圆心画一个半圆到二值图上, 再重扫边界。
 *      补出来的弧位置不好控制, 默认关掉。
 *
 *  方案二: 直接用"边界 - 标准赛宽"算中线 (默认开)
 *      状态3 车已经贴到环岛口: 右边界的搜索终点正好落在环岛的外沿上,
 *      所以 Right_Line 就是环岛的左沿。从它往左退半个赛宽,
 *      就是"环岛左侧那条通道的正中间" —— 正是车该走的位置。
 *      左边界的补线这一步不可靠, 所以干脆不用它。
 *
 *      实现放在 Calculate_Mid_Line() 里, 用 Island_State==3 作为开关。
 * ========================================================================================== */
#define ISL_ARC_ENABLE      0       /* 1 = 开半圆补线, 0 = 关(默认) */

/* 中线 = 边界 干 Base_Width[] * 这个系数
   0.5 = 正常中线(退半个赛宽, 走通道正中)
   想要车离环岛更远(更靠左)就加大, 更贴环岛就减小 */
#define ISL_MID_FACTOR      0.35f

/* ---- 环岛状态 (0~9, 含义见 island.c 里 Island_Detect 的注释) ---- */
extern volatile int Island_State;
extern volatile int Left_Island_Flag;       /* 1 = 判定为左环岛 */
extern volatile int Right_Island_Flag;      /* 1 = 判定为右环岛 */
extern volatile int Left_Up_Guai[2];        /* 左上拐点 [0]=行 [1]=列 */
extern volatile int Right_Up_Guai[2];       /* 右上拐点 [0]=行 [1]=列 */
extern volatile int Island_Over_Flag;       /* 1 = 本帧刚跑完一个环岛(给菜单闪一下用) */

/* 入口判据的中间量, 菜单1 显示用 */
extern volatile int Island_Dbg_CL;          /* Continuity_Change_Left  0=连续 非0=断线行号 */
extern volatile int Island_Dbg_CR;          /* Continuity_Change_Right */
extern volatile int Island_Dbg_ML;          /* Monotonicity_Change_Left  0=单调 */
extern volatile int Island_Dbg_MR;          /* Monotonicity_Change_Right */

/* ---- 环岛状态1 诊断: 锁存快照, 菜单4 显示 ----
   Island_Dbg_ML/MR 每帧开头都会被状态0 的判据结果覆盖, 只有处于状态1 的那几帧
   才是状态1 的搜索结果 —— 屏幕刷新跟帧率同步, 状态1 又可能只有几帧,
   所以直接盯菜单1 的 ML/MR 基本只能看到状态0 的值(直道上就该是 0), 抓不到状态1。

   下面这几个只在 Island_Detect3() 的状态1 里写, 不随帧覆盖, 发车时清零。 */
extern volatile int Isl_Diag_S1_L;      /* 左环岛状态1: 突变点搜索结果, 0=没找到 */
extern volatile int Isl_Diag_S1_R;      /* 右环岛状态1: 突变点搜索结果 */
extern volatile int Isl_Diag_S1_DUR;    /* 最近一次状态1 的持续帧数, 99=卡住 */
extern volatile int Isl_Diag_S1_SS;     /* 状态1 那一刻的 Search_Stop_Line */
extern volatile int Isl_Diag_S1_TOP;    /* 扫描上界, >=15 说明范围被夹没了 */
extern volatile int Isl_Diag_S1_LR;     /* 1=左环岛 2=右环岛 */
extern volatile int Isl_Diag_S1_BS;     /* 那一刻本侧的 Boundry_Start, 1->2 看它<=30 */
extern volatile int Isl_Diag_S1_LT;     /* 那一刻本侧的丢线帧数 */
extern volatile int Isl_Diag_S1_WHY;    /* ★搜索失败原因码: 1丢线 2夹取 3全平坦 4幅度不够 5找到 */
extern volatile int Isl_Diag_S1_HIT;    /* 整个状态1 期间搜到过几帧, 0=一帧都没有 */

/* ★ 搜索窗(第 7~15 行)本侧边界的原始值, 进状态1 第一帧抄下来的 ★
   [0] = 第 7 行, [8] = 第 15 行。搜不到的原因看这 9 个数最直接 */
#define ISL_BAND_N      9
extern volatile int Isl_Diag_S1_BAND[ISL_BAND_N];

/* 单调性搜索最近一次的原因码(全局, 见 island.c 里的说明) */
extern volatile int Isl_Mono_Why;

/* ---- 环岛补线标记 ----
   补过的行置 1。作用等同十字的 Cross_Flag: 给 Calculate_Mid_Line 的情况2b
   和 TFT180_Show_Track_Lines 开门 —— 没有它, 补线写进 Left_Line/Right_Line
   之后会被 Left_Lost_Flag 这道闸门挡住, 等于白补。
   清零在 Longest_White_Column() 里。详见 island.c 的说明。 */
extern volatile uint8 Isl_Patched[IMG_ROW];
void Isl_Patch_Mark(int y1, int y2);

/* ---- 状态3: 锁定最长白列的搜索范围 ----
   (原工程 img_process.c 里本来就有这段, 只是被注释掉了; 数值按 188 -> 94 减半)

   这两个数不只是"视野范围", 它是【边界搜索的种子列】:
       Longest_White_Column_Right[1] / _Left[1] 从这段范围里选出,
       再以它为起点往两边找白黑跳变, 才得到 Left_Line / Right_Line。
   车到环岛口时最长白列可能落在图像左侧(穿过环岛开口), 种子列一偏,
   提出来的"边界"就不是环岛外沿, 角点判据跟着废掉。
   把范围压到右侧, 种子列被逼到环岛那一侧, 信号就干净了。

   实车调整: 角点还是找不稳就动这四个数, 改完自动生效(状态3 每帧都会重扫)。 */
/* ★★★ 图像宽度是 IMG_COL = 94, 不是 120 ★★★
   120 是 MT9V03X 的原生【高度】(188x120)。
   本工程 2:1 抽样之后是 94x60 —— 见 cam_preprocess.c:
       jump_column = MT9V03X_W / IMG_COL = 188 / 94 = 2
   所以列号一律按 94 算; 写超了会被下面那道"防止越界"的钳位砍掉。

   ★ 左右对称 ★
       右环岛: 锁图像最右侧那一段        [68, 91]
       左环岛: 锁它关于图像中心的镜像     [2, 25]    (= 93-91 .. 93-68)

   左环岛的上下界是用右边的值镜像算出来的 —— 以后只调右边那两个,
   左边会自动跟着对称变, 不会调歪。
   最边上两列(92、93)本来就参与不了搜索(i±2 会越界), 所以上界取 IMG_COL-3。 */
#define ISL3_S3_COL_R_START     68
#define ISL3_S3_COL_R_END       (IMG_COL - 3)

/* 上面两个关于图像中心的镜像。IMG_COL-1-x 就是列 x 的对称列 */
#define ISL3_S3_COL_L_START     (IMG_COL - 1 - ISL3_S3_COL_R_END)
#define ISL3_S3_COL_L_END       (IMG_COL - 1 - ISL3_S3_COL_R_START)

/* ---- 偏航角 ----
   本工程没有陀螺仪, FJ_Angle 用左右轮速差积分得到(死推), 不是真陀螺仪。
   原作者用陀螺仪, 三个角度阈值 60/200/320 直接沿用, 但精度不如陀螺仪。 */
extern volatile float FJ_Angle;
extern float Island_Yaw_Gain;               /* 标定系数: 度 / (每10ms的左右轮计数差) */

/* 入环确认帧数: 判据要连续成立这么多帧才认定是环岛。
   用来挡住"斜入十字时先看到一个角"的误判。菜单3 的 "ISL CFM" 就是改这个。 */
extern volatile int Isl_Confirm_Frames;

void  Island_Yaw_Update(int speed_l, int speed_r);  /* TIM6 每 10ms 调用一次 */
void  Island_Reset(void);                           /* 发车时复位 */
void  Island_Detect(void);                          /* 每个摄像头帧调用一次(原版) */

/* 新版: 只把右环岛的 状态1/2 和 2->3 判据换掉, 其余委托给原版。
   main.c 里把 Island_Detect(); 换成 Island_Detect2(); 即可 */
void  Island_Detect2(void);

/* ★ 版本3: 原作者 188x120 那版 Island_Detect() 的完整移植, 全部状态重写 ★
   (左右环岛的 0~8 状态全部按原版逻辑, 数值按 2:1 缩到本工程的 94x60)
   和原版只有两处不同, 原因写在 island.c 里对应位置:
       1. 入环时清零 FJ_Angle  —— 本工程没陀螺仪, 不清零角度阈值全错
       2. 右环岛状态4 用 abs(FJ_Angle) —— 原版只写了 FJ_Angle>200, 符号反了就卡死
   main.c 里把 Island_Detect(); 换成 Island_Detect3(); 即可 */
void  Island_Detect3(void);

/* ---- 四个单边拐点检测, 返回拐点所在行号, 找不到返回 0 ---- */
int Find_Left_Down_Point (int start, int end);
int Find_Left_Up_Point   (int start, int end);
int Find_Right_Down_Point(int start, int end);
int Find_Right_Up_Point  (int start, int end);

/* 同上, 但自己指定形状阈值。状态3 的补线走这条 —— 全局的 ISL_CORNER_*
   是给状态0 的下角点调稳的; 状态3 的角点落在靠上的行、边界很平缓,
   用那套严阈值根本找不到(实测表现: 走兜底, 补线顺着边界延下去)。
   参数含义和可调项见 island.c 的 ISL3_S3_UP_* */
int Find_Left_Up_Point_Ex (int start, int end, int stable, int c2, int c3, int c4);
int Find_Right_Up_Point_Ex(int start, int end, int stable, int c2, int c3, int c4);

/* ---- 连续性(数撕裂点): 连续返回 0, 不连续返回断线处行号 ---- */
int Continuity_Change_Left (int start, int end);
int Continuity_Change_Right(int start, int end);

/* ---- 单调性突变: 返回突变点行号, 找不到返回 0 ---- */
int Monotonicity_Change_Left (int start, int end);
int Monotonicity_Change_Right(int start, int end);

/* 同上, 但可以指定"顶部豁免行数"(普通版固定用 ISL_TOP_MARGIN=8)。
   ★ 状态1/2 的补线必须用 top_skip=0 ★
   top_skip=8 时, 第 5~15 行这种靠上的搜索范围会被 isl_clamp_range
   夹成 start==end 然后返回 0, 补线永远画不出来。
   详见 island.c 里 Monotonicity_Change_Left_Ex 的注释。 */
int Monotonicity_Change_Left_Ex (int start, int end, int top_skip, int prom);
int Monotonicity_Change_Right_Ex(int start, int end, int top_skip, int prom);

/* ---- 补线 ---- */
void K_Draw_Line        (float k, int startX, int startY, int endY);
void K_Add_Boundry_Left (float k, int startX, int startY, int endY);
void K_Add_Boundry_Right(float k, int startX, int startY, int endY);
void Draw_Line          (int x1, int y1, int x2, int y2);

/* ---- 工具 ---- */
int   Get_Road_Wide(int start_line, int end_line);
float Get_Left_K   (int start_line, int end_line);
float Get_Right_K  (int start_line, int end_line);

#endif
