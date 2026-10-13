#include "zf_common_headfile.h"
#include "Ourcode_headfile.h"
#include "island.h"
#include <math.h>

/* ==========================================================================================
 *  环岛识别
 *
 *  来源: 开源工�?Island.c (原作�?Joshua.Xu, �?188x120)
 *  本工程图 94x60, 两个方向都正�?1/2
 *
 *  【换算原�?—�?这个必须分清�? 搞错了不是精度问题而是数组越界�? *
 *      �?物理�?(要跟着分辨率缩�?
 *         行号 / 列号 / 行数 / 列数      ->  原�?/ 2
 *         单调性判断的窗口 ±5 �?         ->  ±2 �? *         撕裂点阈�?5 �?                ->  3 �? *         角点突变阈�?5/8/10/15 �?      ->  3/4/5/8 �? *
 *      �?数组安全�?(绝对不能缩放!)
 *         角点判据里要访问 Left_Line[i±4],
 *         所以行号起点最大只能到 IMG_ROW-1-4, 终点最小只能到 4�? *         原作者留的是 5, 这里保留 4~5 的余量�? *         如果把这个余量也减半�?2, i+4 会直接越界读到别的变量�? *
 *      �?角度 (不能缩放)
 *         FJ_Angle �?60 / 200 / 320 是度, 和图像分辨率无关
 *
 *  实车调参: 下面这些宏就是全部可调项, 改这里即可�? * ========================================================================================== */

/* ---------------------------------------------------------------- 数组安全�?   这几个是"防止数组越界"用的, 和图像分辨率无关, 绝对不能跟着减半! */
#define ISL_TOP_ROWS        5       /* 绝对行号下限, �?cam_preprocess.h �?IMG_TOP 保持一�?*/
#define ISL_IDX_MARGIN      4       /* 角点判据里最大要访问 Left_Line[i±4], 首尾各留 4 �?*/

/* �?甩掉"赛道最远处"那几�?�?   最长白列刚刚够到的地方就是图像里赛道的最远处, 那里只有几个像素�?
   二值化在白/黑之间反复跳 -> 边界数据在跳 -> 单调性判据会在这里找到一个假转折点�?   实测: SS=55、车摆在直道上时, ML 会稳定读�?7
         (= IMG_ROW - SS + ISL_WIN), 正好是扫描范围顶�? 就是这个问题�?
   这里�?从有效区顶端往下再甩掉几行", 不是绝对行号, 所�?SS 变了也不用重调�?   连续性和单调性判断用�? 角点判断不用(�?isl_clamp_range �?top_skip 参数)�?
   调法: 直道�?ML/MR 还有�?0 就继续加�? 加到环岛判不出来(有效判断区太�?就往回调�?*/
#define ISL_TOP_MARGIN      8

/* ---------------------------------------------------------------- 物理阈�?*/
#define ISL_WIN             2       /* 单调性窗�?[�?±5 -> ±2] */

/* �?单调性判据的"幅度要求" �?   原来�?isl_is_local_max/min 只要�?�?±ISL_WIN 行里是最�?最�?, 不要求大多少�?   于是边界上一�?1~3 像素的毛刺就能被当成"单调性突变点"�?   实测: 车在直道上左右摆动时 ML/MR �?0 <-> 五十�?来回�?
         根源是赛道直线在图像里本来就有几个像素不�?(二值化的毛�?�?
   这里要求这个点比窗口两端最少高�?ISL_MONO_PROM 个像素才算转折�?   算例: 边界正常每行�?1 像素, 某个毛刺让第 i 行凸�?3 像素
         -> 旧的判据成立(报出这个行号), 新的判据 line[i]-line[i-2] 只有 1 < 3, 被拦住�?         而真实的单调性突变幅度有十几二十像素, 照样通过�?
   �?设成 0 就完全等于原来的行为 �?—�?这个改动是纯加法, 不会破坏已经调好的东西�?
   调法: 直道上摆动车还跳就加�?(3 -> 5); 如果环岛跑到状�?4 就不动了
         (真实的转折点也被滤掉�?, 就往回调�?*/
#define ISL_MONO_PROM       3

#define ISL_TEAR            3       /* 撕裂点阈�?�? [�?5 -> 3] */
#define ISL_CORNER_STABLE   3    /* 角点稳定阈�?�? [�?5 -> 3] */
#define ISL_CORNER_C2       3       /* 角点突变阈�?[�?5  -> 3] */
#define ISL_CORNER_C3       5       /* 角点突变阈�?[�?8  -> 4, �?5 更稳] */
#define ISL_CORNER_C4       8       /* 角点突变阈�?[�?10 -> 5, �?8 更稳] */

/* 入环判定时的丢线数范�?[�?10~50 -> 5~25] */
#define ISL_LOST_MIN        5
#define ISL_LOST_MAX        25
/* 入环判定时另一侧允许的丢线�?[�?10 -> 5] */
#define ISL_LOST_OTHER      5

/* 入环判定要求"能看到多�? (最长白列要够长)
   实物对照: 菜单1 �?8 行的 SS 就是这个�? 满值是 IMG_ROW(60)

   原作者用的是 0.95 (=57), 那是配他 188x120 的摄像头和俯仰角的�?   本车实测: 直道 SS=55, 环岛入口 SS=53 —�?直道反而更�?
   也就是说这条判据在本车上区分不了环岛和直�? 只会挡路, 所以降�?0.80 (=48)�?   实测�?53 > 48, 能过�?
   要调的话改这个比例就行�?*/
#define ISL_STOP_RATIO      0.80f
#define ISL_STOP_MIN        ((int)(IMG_ROW * ISL_STOP_RATIO))

/* 入环判定时的边界起始点要�?[�?IMG_ROW-20 -> IMG_ROW-10] */
#define ISL_START_NEAR      (IMG_ROW - 10)
/* 出环判定时的边界起始点要�?[�?IMG_ROW-10 -> IMG_ROW-5] */
#define ISL_START_VERY_NEAR (IMG_ROW - 5)

/* 入口判定: 下角�?Find_*_Down_Point)的行号必�?>= 这个, 否则认为是误�?   [�?30 -> 15]。调�?= 入口更难成立 */
#define ISL_DOWN_GONE       15

/* �?入环确认帧数 �? *
 *  斜着进十字的时�? 车会先看到十字的一个角 —�?那一两帧的图像特�? *  (一侧不连续 + 下角�? 和环岛入口几乎一模一�? 直接判定就会误入状�?�? *  结果: 车速被压到 Island_Speed, 而且环岛状态机整个跑偏�? *
 *  真环岛的特征�?这个特征会持续存�?, 十字的角只是路过�? *  所以要求上面那一整套判据连续成立 Isl_Confirm_Frames 帧才真的认定�? *
 *  4 �?@50fps = 80ms。调�?= 更不容易误判, 但入环判定也更晚�? *  十字误判还是多就加到 6; 觉得入环太晚就减�?2~3�? *  �?菜单3 �?"ISL CFM" 就是改这�?�?*/
volatile int Isl_Confirm_Frames = 10;

/* 状�? 补线�? 单调性搜索的起点 [�?30 -> 15]
   �?注意: 这原来和 ISL_DOWN_GONE 共用同一个宏, 但两者的方向是相反的
     (入口�?角点行号 >= x", 出口�?BS <= x"), 共用会导致调一个坏一�? 所以拆开 */
#define ISL_DOWN_SEARCH     15

/* �?状�? -> 2 的出�? Boundry_Start_* 掉到这个行号以下, 说明下方丢线�?�?   原作者用 30 (120 行图), 按比例换�?60 行应该是 15 —�?但那是错的�?   实测: 车停在环岛口�?BS=59; 往前开、右下角点消失的一瞬间 BS 掉到 28�?   所以这里直接用实测�?30, 不要按分辨率缩放�?   (缩放的只�?行号/列号"这类几何�? 这个门槛反映的是"丢线区往上延伸到多高",
    取决于赛道和摄像�? 只能实测) */
#define ISL_DOWN_BS         30
/* 状�? 寻找弧线的起�?[�?70 -> 35] */
#define ISL_ARC_SEARCH      35
/* 状�? 判断"圆弧靠下�?的行�?[�?50 -> 25] */
#define ISL_ARC_LOW         25

/* 状�? 找上拐点的范�?[�?40~5 -> 20~2] */
#define ISL_UP_SEARCH_HI    20
#define ISL_UP_SEARCH_LO    2
/* 状�? 拐点位置是否合理的判�?[�?15~50 �? 50~110 �?-> 7~25 �? 25~55 列] */
#define ISL_UP_ROW_MIN      7
#define ISL_UP_ROW_MAX      25
#define ISL_UP_COL_MIN      25
#define ISL_UP_COL_MAX      55
/* 状�? 拐点行号小于这个就认为误�?[�?5 -> 2] */
#define ISL_UP_BAIL         2

/* 状�? 找单调性突变点的范�?[左环 �?90 -> 45, 右环 �?IMG_ROW-10 -> IMG_ROW-5] */
#define ISL_MONO_FAR        45
/* 状�? 突变点行号是否合理的判据 [�?35~55 -> 17~27] */
#define ISL_MONO_ROW_MIN    17
#define ISL_MONO_ROW_MAX    27
/* 状�? 突变点列坐标门限 [�?>=10 -> >=5] */
#define ISL_MONO_COL_MIN    5

/* 状�? 找拐点的范围 [�?IMG_ROW-10~0 -> IMG_ROW-5~0] */
#define ISL_EXIT_SEARCH     (IMG_ROW - 5)
/* 状�? 左环拐点列坐标上�?[�?<=100 -> <=50] */
#define ISL_EXIT_COL_L      50
/* 状�? 右环拐点列坐标下�?[�?IMG_W-88 -> IMG_W-44] */
#define ISL_EXIT_COL_R      (IMG_COL - 44)
/* 状�? 拐点行号上限 [�?IMG_ROW-20 -> IMG_ROW-10] */
#define ISL_EXIT_ROW_MAX    (IMG_ROW - 10)
/* 状�? 拐点行号下限 [�?5 -> 2] */
#define ISL_EXIT_ROW_MIN    2

/* 状�? 找拐点的范围 [�?IMG_ROW-1~10 -> IMG_ROW-1~5] */
#define ISL_EXIT_SEARCH_LO  5
/* 状�? 判断"拐点丢了但下方不丢线"的行�?[�?10 -> 5] */
#define ISL_EXIT_LOST_ROW   5

/* ---------------------------------------------------------------- 状�? 半圆补线
   环岛是个�? 车要绕着它走。原来状�? 画的是一条死直线(切线), 车会沿着直线冲出�?
   拐不进环。这里改成以角点为圆心画一个真正的半圆�?
   半径怎么�? 用弧顶那一行的标准赛宽 Base_Width[] 推�?   理由: 环岛的直径大致和赛道宽度同量�? 取半就是半径; 而且 Base_Width 是逐行实测�?
         图像上端的行也有一份可信的�? 比拍脑袋给常数好�?
   �?弧往哪边�? 实车如果拐反了就把这两个宏对�?�?*/
#define ISL_ARC_R_RATIO     0.50f   /* 半径 = 该行标准赛宽 × 这个系数 */
#define ISL_ARC_R_MIN       8.0f    /* 半径下限(像素), 防止标定异常时画成一个点 */
#define ISL_ARC_R_MAX       40.0f   /* 半径上限(像素) */

#define ISL_ARC_DIR_R       (-1)    /* 右环�? 半圆往左张 */
#define ISL_ARC_DIR_L       (+1)    /* 左环�? 半圆往右张 */

/* 补线时用的横向偏�?[�?IMG_W-30 -> IMG_W-15] */
#define ISL_DRAW_X_R        (IMG_COL - 15)
/* 补线时用的横向偏�?[�?30 -> 15] */
#define ISL_DRAW_X_L        15

/* 入环/出环的角度门�?(�? 不缩�? */
#define ISL_ANGLE_IN        80.0f
#define ISL_ANGLE_MID       200.0f
#define ISL_ANGLE_OUT       320.0f

/* ---------------------------------------------------------------- 状态变�?*/
volatile int Island_State       = 0;
volatile int Left_Island_Flag   = 0;
volatile int Right_Island_Flag  = 0;
volatile int Left_Up_Guai[2]    = {0, 0};
volatile int Right_Up_Guai[2]   = {0, 0};
volatile int Island_Over_Flag   = 0;

/* 状�? 的半圆补�? 弧顶(角点)坐标 + 是否已经锁定�?   做成文件级静态而不�?Island_Detect 的局部静�? 是为了让 Island_Reset() 也能清掉�?*/
static int   island_arc_row = 0;
static int   island_arc_col = 0;
static uint8 island_arc_on  = 0;

/* 环岛入口判据的中间量, 只为菜单显示�?不然屏幕上没法看出卡在哪一�?�?   �?Island_Detect 开头更�? 含义�?IsLand_Detect 里用的一模一�?
       Dbg_CL / Dbg_CR : Continuity_Change_Left/Right 的结�? 0=连续, �?=断线处行�?       Dbg_ML / Dbg_MR : Monotonicity_Change_Left/Right 的结�? 0=单调, �?=突变点行�?*/
volatile int Island_Dbg_CL = 0;
volatile int Island_Dbg_CR = 0;
volatile int Island_Dbg_ML = 0;
volatile int Island_Dbg_MR = 0;

/* ==========================================================================================
 *  ★ 环岛状态1 诊断 —— 锁存快照, 菜单4 显示 ★
 *
 *  为什么必须锁存:
 *      Island_Dbg_ML / Island_Dbg_MR 在每个摄像头帧的开头都会被"状态0 的判据结果"
 *      覆盖一次, 只有真正处于状态1 的那几帧才会被状态1 的搜索结果覆盖。
 *      屏幕刷新和摄像头帧同步, 而状态1 可能只有几帧 ——
 *      所以直接盯菜单1 的 ML/MR, 看到的几乎永远是状态0 的值(直道上就该是 0),
 *      根本抓不到状态1 那一刻的搜索结果。这不是状态机的问题, 是显示方式的问题。
 *
 *  下面这几个只在状态1 里写, 平时不覆盖, 发车时由 Island_Reset() 清零:
 *      S1_L / S1_R  状态1 单调突变点的搜索结果。0 = 没找到(补线不会画);
 *                   非 0 就是突变点行号, 补线会画出来
 *      S1_DUR       最近一次状态1 持续了多少帧(每次"进入状态1"那一帧清零重数)。
 *                   几十 = 正常穿过; 99 = 卡死在状态1 出不来
 *      S1_SS        那一刻的 Search_Stop_Line
 *      S1_TOP       isl_clamp_range 夹出来的扫描上界。>= ISL3_S1_SEARCH 就说明
 *                   整个扫描范围被夹没了, 搜索结果必然是 0
 *      S1_LR        1 = 当时是左环岛, 2 = 右环岛
 *      S1_BSL/BSR   那一刻的 Boundry_Start_Left / Right。
 *                   状态1->2 的条件就是本侧 BS <= ISL_DOWN_BS(30),
 *                   所以这两个数直接告诉你"还差多少才肯进状态2"
 *      S1_LT        那一刻"本侧"的丢线帧数(左环岛看 Left_Lost_Time, 右环岛看 Right)。
 *                   ★ 重点看这个 ★ 如果它 >= 9, 说明 7~15 行整段都丢线,
 *                   Left_Line[7..15] 全是 0 -> isl_is_flat 判定"平坦"整段跳过
 *                   -> 单调突变点必然搜不到 -> 补线必然画不出来
 * ========================================================================================== */
/* 状态1 补线搜索的参数与观测窗。
   ISL3_S1_SEARCH 就是 _Ex(ISL3_S1_SEARCH, 2, 0) 那个起点行(原版 30 减半) */
#define ISL3_S1_SEARCH      15
/* 状态1/2 补线找单调突变点时的"幅度要求"。
   ★ 必须用 0 ★ —— 原版算法没有这个要求, ISL_MONO_PROM(=3) 是本工程为
   状态0 的判据后加的。状态1/2 只在环岛已经确认之后才跑, 不存在误判环岛的风险,
   而 幅度>=3 会把真实拐点一起滤掉(实测搜不到就是这个原因)。
   用 0 时完全等于原版行为。 */
#define ISL3_PATCH_PROM     0

/* 状态2 单调突变点搜索的起点行。
   原版是 70(120 行图), 按分辨率减半是 35 —— 但 35 会让搜索结果永远到不了
   下面那个 ISL3_S2_EXIT_ROW(=45), 出口判据就永远不成立, 所以放宽到 55。
   实际起点还会被压到 Boundry_Start_* - 1 以内(只能搜有效边界行), 见状态2 注释。 */
#define ISL3_S2_SEARCH      55

/* ★ 状态2 -> 3 的出口判据 ★
   单调突变点的行号大于这个值就进状态3。
   行号越大 = 越靠近车(画面下方)。45 = 画面下方约 3/4 处。
   调大 = 更晚进状态3, 调小 = 更早进。 */
#define ISL3_S2_EXIT_ROW    45

/* 状态3 找角点的"窗口"。
   ★ 锁定最长白列之后, 要找的角点左右会翻过来 ★
       最长白列锁在右侧 -> 提出来的"左边界"其实就是环岛的外沿
                         -> 要找【左上】角点, 用 Left_Add_Line 补线
       最长白列锁在左侧 -> 对称, 要找【右上】角点, 用 Right_Add_Line 补线
   (文章原话: "强制将最长白列锁定在右侧, 这样的话右上角点就会变成左上角点")

   角点落在窗口内就补线; 跑出窗口(靠下 = 行号太大, 或靠左 = 列号太小)
   就停止补线, 但锁定保留, 让巡线自己继续找边界。

   行方向的搜索范围/窗口左右环岛共用 ISL_UP_SEARCH_HI/LO 和 ISL_UP_ROW_MIN/MAX,
   只有列窗口要分开(两边的几何不对称)。

   ★ 列窗口为什么放到这么宽 ★
   原来是 35~75 / 19~59, 那是按【旧的宽锁定范围(47~84 / 10~47)】定的。
   锁定收到最右侧 [68,91] 之后, 种子列被赶到 >=68, 从种子【往左】找出来的
   边界(也就是角点所在的列)会落在很靠左的位置 —— 旧的 35 下界会把这种角点
   直接拒掉, 表现就是"角点明明找到了, 补线却不出"，而且一声不吭。

   现在只排除最边上几列(那几列本来就不可信), 过滤交给:
       - 行窗口 ISL_UP_ROW_MIN/MAX
       - 角点自身的形状判据(上面4行平 + 下面4行内降 8 像素)
   这两条已经够严了, 列窗口不需要再当第二道闸门。 */
#define ISL3_S3_R_COL_MIN   5
#define ISL3_S3_R_COL_MAX   (IMG_COL - 6)
#define ISL3_S3_L_COL_MIN   5
#define ISL3_S3_L_COL_MAX   (IMG_COL - 6)

/* ★★ 状态3 找角点的【形状判据】—— 单独一套, 不复用全局的 ISL_CORNER_* ★★

   为什么要分开:
     ① ISL_CORNER_* 是 4 个角点函数共用的, 其中 Find_*_Down_Point 是
        状态0 认定环岛的判据之一 —— 全局放松会把入环判据一起弄松。
     ② 更重要的是方向不同: 状态0 找的"下角点"在图像中下部, 边界在那里
        每行变化大, 用严阈值合适; 而状态3 锁定最长白列之后, 角点落在
        靠上的行(接近地平线), 边界每行只变一两个像素 ——
        用状态0 那套严阈值【根本找不到】。
        实测表现就是"走兜底, 补线顺着边界延下去"。

   四个可调项(实测调这几个):
     ISL3_S3_UP_HI      搜索起始行 —— 往下能扫到多低。
                        角点比这个还靠下就永远找不到, 这是第一个要动的
     ISL3_S3_UP_STABLE  "上面几行要平"的容差, 调大更容易通过
     ISL3_S3_UP_C2/C3/C4  "下面 2/3/4 行内边界要往左降多少像素", 调小更容易通过 */
#define ISL3_S3_UP_HI       35      /* 原版 40 -> 20, 这里放宽到 35 */
#define ISL3_S3_UP_STABLE   3
#define ISL3_S3_UP_C2       2
#define ISL3_S3_UP_C3       3   
#define ISL3_S3_UP_C4       4

#define ISL3_BAND_N         9                               /* 观测窗行数 */
#define ISL3_BAND_ROW0      (ISL3_S1_SEARCH - (ISL3_BAND_N - 1))    /* = 7, 窗是 7~15 行 */

volatile int Isl_Diag_S1_L   = 0;
volatile int Isl_Diag_S1_R   = 0;
volatile int Isl_Diag_S1_DUR = 0;
volatile int Isl_Diag_S1_SS  = 0;
volatile int Isl_Diag_S1_TOP = 0;
volatile int Isl_Diag_S1_LR  = 0;
volatile int Isl_Diag_S1_BS  = 0;
volatile int Isl_Diag_S1_LT  = 0;
volatile int Isl_Diag_S1_WHY = 0;

/* 状态1 期间搜索成功的帧数。0 = 整个状态1 里一帧都没搜到过 */
volatile int Isl_Diag_S1_HIT = 0;

/* ★ 决定性的观测窗 ★
   进入状态1 的第一帧, 把搜索窗(第 7~15 行)本侧的边界原值原样抄下来。
   搜索为什么失败, 看这 9 个数就一目了然:
       全是同一个数        -> 平坦, WHY 会是 3
       大部分是 0          -> 那几行根本没提出边界(丢线)
       有起伏但很平缓      -> 幅度不到 ISL_MONO_PROM, WHY 会是 4
   [0] 对应第 ISL3_BAND_ROW0(7) 行, [8] 对应第 15 行 */
volatile int Isl_Diag_S1_BAND[ISL3_BAND_N] = {0};

/* ==========================================================================================
 *  ★ 环岛补线标记 (Isl_Patched) —— 本工程一个结构性的坑, 必须理解 ★
 *
 *      Left_Add_Line() / Right_Add_Line() 只改 Left_Line[] / Right_Line[],
 *      不会去动 Left_Lost_Flag[] / Right_Lost_Flag[]。
 *      而下游有两处都拿 Lost_Flag 当闸门:
 *
 *          Calculate_Mid_Line()       left_valid = (Left_Lost_Flag[i] == 0)
 *          TFT180_Show_Track_Lines()  left_valid = (Left_Lost_Flag[i] == 0 || Cross_Flag == 1)
 *
 *      问题是: 补线补的恰恰就是"丢线的那几行", 而那几行正好被闸门挡着 ——
 *      补了等于没补, 屏幕上也不会画出来。
 *
 *      Cross_Detect() 之所以看起来有效, 是因为它补完线以后 Cross_Flag 会置 1,
 *      Calculate_Mid_Line 的情况2 和显示函数都专门给它开了门。
 *
 *      环岛没有这个标志 —— 所以本工程里环岛的补线
 *      (Island_Detect 状态1/2 的, 以及 Island_Detect3 状态1/2 的)
 *      从来就没真正生效过。只是状态1/2 一闪而过, 一直没被发现。
 *
 *      这里补上完全对称的机制: 谁往边界数组里补了线, 就把补过的行标上,
 *      Calculate_Mid_Line 的情况2b 和显示函数都会给这些行开门。
 *
 *  清零时机: Longest_White_Column() 每次重建边界数组时一起清 ——
 *            边界都重新提取了, 旧的补线标记自然作废。
 * ========================================================================================== */
volatile uint8 Isl_Patched[IMG_ROW] = {0};

/* 把 [y1, y2] 这些行标成"补过的" */
void Isl_Patch_Mark(int y1, int y2)
{
    int i, a, b;

    a = (y1 < y2) ? y1 : y2;
    b = (y1 > y2) ? y1 : y2;

    if (a < 0)           { a = 0; }
    if (b > IMG_ROW - 1) { b = IMG_ROW - 1; }

    for (i = a; i <= b; i ++)
    {
        Isl_Patched[i] = 1;
    }
}

/* ------------------------------------------------------------------------------------------
 *  单调性搜索"走到哪一步就返回了"的原因码, 由 isl_mono_change_left/right 填写。
 *
 *  ★ 这个才是决定性的诊断 ★ 它直接说明搜不到的原因, 不用再靠猜:
 *      0 = 还没搜过
 *      1 = 本侧丢线太多(>= 0.9*IMG_ROW), 函数一进来就返回了
 *      2 = 扫描范围被 isl_clamp_range 夹没了(start <= end)
 *      3 = 范围内每一行都是"平坦"的(isl_is_flat), 整段被 continue 跳过
 *          -> 典型情况就是那几行的边界值全是 0(整段丢线), 或者真的完全水平
 *      4 = 范围内有起伏, 但没有任何一行的"局部极大"幅度够 ISL_MONO_PROM(=3)
 *          -> 阈值太严, 真实拐点被当毛刺滤掉了
 *      5 = 找到了
 * ------------------------------------------------------------------------------------------ */
volatile int Isl_Mono_Why = 0;

/* ---------------------------------------------------------------- 偏航�?   原作者用陀螺仪积分。本工程没有陀螺仪, 改用左右轮速差积分(死推):
       车头角速度 �?(右轮速度 - 左轮速度) / 轮距
   把轮距、轮径、计数当量、时间都折进一个系�?Island_Yaw_Gain,
   单位就是"�?/ (�?10ms 的左右轮计数�?"�?
   标定方法:
       1. 先把 FJ_Angle 显示到菜单上(菜单1 �?ISL 那几�?
       2. �?Island_Yaw_Gain = 1.0
       3. 把车放在地上, 用手按住一侧轮�? 推另一�? 让车原地正好�?360 �?       4. �?FJ_Angle 读数 A
       5. Island_Yaw_Gain = 1.0 * 360 / A     (A 是负数就把符号一起带进去)

   注意: 这是"穷人版陀螺仪"。车轮打滑、原地空转、速度突变时都会失�?
         但环岛只拿它卡阶�?60/200/320 �?, 精度够用�?   如果标定�?FJ_Angle 一直是 0, 先检�?Island_Yaw_Update 有没有在 TIM6 里被调用�?*/
volatile float FJ_Angle        = 0.0f;
/* 实车标定�? 推车�?180 �? ANG 读到 2800 -> Gain = 180/2800 = 0.0643
   理论核对: 180 度转弯两后轮路程�?= pi * 轮距, 你这边约 5000 计数/�?
             反推轮距 = 2800/(pi*5000) = 0.178m, 正是后轮距的典型�? 数据可信�?   换车/换轮�?换编码器都要重新标�?*/
float          Island_Yaw_Gain = 0.0643f;

/* 原作者图里的几个�? 这里统一映射过来 */
extern volatile int   Left_Line[IMG_ROW];
extern volatile int   Right_Line[IMG_ROW];
extern volatile int   Search_Stop_Line;
extern volatile int   Boundry_Start_Left;
extern volatile int   Boundry_Start_Right;
extern volatile int   Left_Lost_Time;
extern volatile int   Right_Lost_Time;
extern volatile int   Both_Lost_Time;
extern volatile int   Cross_Flag;
extern volatile int   Ramp_Flag;

/* ==========================================================================================
 *                                          工具函数
 * ========================================================================================== */

/*-------------------------------------------------------------------------------------------------------------------
  @brief    �?[start, end] 收进安全范围
  @param    idx_margin  判据里最大要访问 i±idx_margin, 首尾各留这么多余�?  @note     这个余量�?数组安全"用的, 和图像分辨率无关, 绝对不能跟着减半
-------------------------------------------------------------------------------------------------------------------*/
static void isl_clamp_range(int *start, int *end, int idx_margin, int top_skip)
{
    int t;
    int top;

    if (*start < *end)                          /* 统一�?start >= end, 都是从下往上扫 */
    {
        t = *start;
        *start = *end;
        *end = t;
    }

    /* 起点不能太靠�? 要给 i+idx_margin 留出数组空间 */
    if (*start > IMG_ROW - 1 - idx_margin)
    {
        *start = IMG_ROW - 1 - idx_margin;
    }

    /* 终点不能太靠�? 要给 i-idx_margin 留空�? 另外顶部 ISL_TOP_ROWS 行本来就不搜 */
    if (*end < idx_margin)      { *end = idx_margin; }
    if (*end < ISL_TOP_ROWS)    { *end = ISL_TOP_ROWS; }

    /* �?关键 �?不要把扫描范围伸�?边界数组还没被赋�?的区域�?       Left_Line[] 每帧先全部清�? 只有 [IMG_ROW-Search_Stop_Line, IMG_ROW-1] 这些�?       会被 Longest_White_Column() 填真�? 再往上的行左边恒�?0、右边恒�?IMG_COL-1�?
       注意这里必须再往上让�?idx_margin �?
       判据在判断第 i 行时还要回头看好几行 (连续性是 i-1, 单调性是 i±ISL_WIN,
       角点判据�?i±4)。如果只让到"有效区最上面一�?, �?i 取到那一行时,
       i-1 / i-2 / i-4 就落到有效区外面, 读到的是没赋值的 0, 于是:
          连续�? abs(真�?- 0) 很大            -> 假撕裂点
          单调�? 真值比上面那一�?0 都大        -> 假局部最大�?          角点:   真�?- 0 >= 阈�?             -> 假拐�?       实测症状: 车摆在直道上 (本该全是 0), 菜单显示 CL=5 CR=5 ML=6 MR=6,
                 这几个数就是"扫描范围顶端"本身, 不是任何真实特征�?*/
    top = IMG_ROW - Search_Stop_Line + idx_margin + top_skip;
    if (top < ISL_TOP_ROWS) { top = ISL_TOP_ROWS; }     /* 顶部总要留一�?*/
    if (top > *start)       { top = *start; }           /* 不能越过起点, 否则会数组越�?*/
    if (*end < top)         { *end = top; }

    if (*start < *end)                          /* 收完之后万一反了, 再换一�?*/
    {
        t = *start;
        *start = *end;
        *end = t;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    判断 line[i] 在前�?ISL_WIN 个点里是不是(不严�?最大�?-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------
  @brief    局部极大/极小判断
  @param    prom  这个点要比窗口两端各高出(或低出)多少像素才算数。
                  传 ISL_MONO_PROM 就是原来的行为; 传 0 就是原版算法
                  (只要在 ±ISL_WIN 行里是最大/最小就算, 不要求幅度)。
  @note     原来 prom 是写死的 ISL_MONO_PROM, 现在开成参数。

            ★ 为什么需要它 ★
            ISL_MONO_PROM 是本工程后加的(原版算法没有幅度要求, 只要在 ±ISL_WIN
            行里是最大/最小就算)。它挡的是"车在直道上左右摆动时 ML/MR 乱跳",
            对状态0 的判据是必要的。

            但状态1/2 的补线是另一回事: 那时环岛已经确认了, 不存在"误判环岛"
            的风险, 而它找的恰恰是真实的单调突变点 —— 要求幅度 >= 3 会把真拐点
            一起滤掉(实测 WHY=4 就是这个)。所以那条路径传 prom = 0,
            完全等于原版行为。
-------------------------------------------------------------------------------------------------------------------*/
static uint8 isl_is_local_max(const volatile int *line, int i, int prom)
{
    int d;

    for (d = 1; d <= ISL_WIN; d ++)
    {
        if (line[i] < line[i + d]) { return 0; }
        if (line[i] < line[i - d]) { return 0; }
    }

    /* 幅度要求: 要比窗口两端都高�?ISL_MONO_PROM 才算转折, 否则只是二值化的毛刺�?       ISL_MONO_PROM = 0 时这两句恒不成立, 完全等于原来的行�?*/
    if (line[i] - line[i + ISL_WIN] < prom) { return 0; }
    if (line[i] - line[i - ISL_WIN] < prom) { return 0; }

    return 1;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    判断 line[i] 在前�?ISL_WIN 个点里是不是(不严�?最小�?-------------------------------------------------------------------------------------------------------------------*/
static uint8 isl_is_local_min(const volatile int *line, int i, int prom)
{
    int d;

    for (d = 1; d <= ISL_WIN; d ++)
    {
        if (line[i] > line[i + d]) { return 0; }
        if (line[i] > line[i - d]) { return 0; }
    }

    /* 幅度要求, �?isl_is_local_max */
    if (line[i + ISL_WIN] - line[i] < prom) { return 0; }
    if (line[i - ISL_WIN] - line[i] < prom) { return 0; }

    return 1;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    判断 line[i] 前后 ISL_WIN 个点是不是全都一�?一段平�? 不能当转折点)
-------------------------------------------------------------------------------------------------------------------*/
static uint8 isl_is_flat(const volatile int *line, int i)
{
    int d;

    for (d = 1; d <= ISL_WIN; d ++)
    {
        if (line[i] != line[i + d]) { return 0; }
        if (line[i] != line[i - d]) { return 0; }
    }
    return 1;
}

/* ==========================================================================================
 *                                     连续�?/ 单调�? * ========================================================================================== */

/*-------------------------------------------------------------------------------------------------------------------
  @brief    3 点中�? 返回 line[i-1]、line[i]、line[i+1] 的中位数
  @note     专门用来滤掉"单行毛刺"�?            赛道直线在图像里偶尔有几个像素不�? 表现�?Left_Line[i] 单独一行凸出去
            几个像素、下一行又回来。这种单点毛刺在连续性判据里会被当成"撕裂�?
            (相邻两行的落�?= 毛刺高度 + 正常斜率, 很容易超�?ISL_TEAR), 于是
            车一左右摆动, CL/CR 就在 0 和那个行号之间来回跳�?
            为什么用中值而不是低�? 真实的撕�?(边界断开、跳到另一条边�? �?            连着好几行的, 中值滤波动不了�? 而单点毛刺正好被中值抹平�?            低通会把真实的撕裂也一起削�? 中值不会�?-------------------------------------------------------------------------------------------------------------------*/
static int isl_med3(const volatile int *line, int i)
{
    int a = line[i - 1];
    int b = line[i];
    int c = line[i + 1];
    int t;

    if (a > b) { t = a; a = b; b = t; }     /* 现在 a <= b */
    if (b > c) { b = c; }                   /* 现在 b = min(b, c) */

    return (a > b) ? a : b;                 /* max(a, min(b,c)) 就是中位�?*/
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    左边界连续性检�?  @return   连续返回 0, 不连续返回断线处的行�?  @note     作者注�? 连续性的阈值设置为5, 可更�?-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Left(int start, int end)
{
    int i;
    int flag = 0;

    if (Left_Lost_Time >= 0.9 * IMG_ROW)        /* 大部分都丢线, 没必要判断了 */
    {
        return 1;
    }
    if (Search_Stop_Line <= 2)                  /* [�?5 -> 2] 搜索截止行太�?*/
    {
        return 1;
    }

    isl_clamp_range(&start, &end, 2, ISL_TOP_MARGIN);   /* 中值滤波要访问 i+1 �?i-2 */

    for (i = start; i >= end; i --)
    {
        /* 先各自做 3 点中值再去比: 单行毛刺被抹�? 连续好几行的真实撕裂保留 */
        if (abs(isl_med3(Left_Line, i) - isl_med3(Left_Line, i - 1)) >= ISL_TEAR)
        {
            flag = i;
            break;
        }
    }
    return flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    右边界连续性检�?  @return   连续返回 0, 不连续返回断线处的行�?-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Right(int start, int end)
{
    int i;
    int flag = 0;

    if (Right_Lost_Time >= 0.9 * IMG_ROW)
    {
        return 1;
    }

    isl_clamp_range(&start, &end, 2, ISL_TOP_MARGIN);   /* 中值滤波要访问 i+1 �?i-2 */

    for (i = start; i >= end; i --)
    {
        if (abs(isl_med3(Right_Line, i) - isl_med3(Right_Line, i - 1)) >= ISL_TEAR)
        {
            flag = i;
            break;
        }
    }
    return flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    左边界单调性突变检�?  @return   突变点行�? 找不到返�?0
  @note     作者注�? �?�?它最�?最�?, 那它就是角点。这里窗口缩�?±ISL_WIN
-------------------------------------------------------------------------------------------------------------------*/
/* top_skip = 顶部豁免行数, 由调用者决定:
   - 状态0 的判据用 ISL_TOP_MARGIN(8), 要甩掉顶部的二值化毛刺;
   - 状态1/2 的补线必须用 0, 原因见 Monotonicity_Change_Left_Ex 的注释。 */
static int isl_mono_change_left(int start, int end, int top_skip, int prom)
{
    int i;
    int ret = 0;
    int saw_slope = 0;                          /* 范围内有没有出现过"不平坦"的行 */

    Isl_Mono_Why = 0;

    if (Left_Lost_Time >= 0.9 * IMG_ROW)
    {
        Isl_Mono_Why = 1;
        return ret;
    }

    isl_clamp_range(&start, &end, ISL_WIN, top_skip);

    if (start <= end)                           /* 递减计算, 入口反了 */
    {
        Isl_Mono_Why = 2;
        return ret;
    }

    for (i = start; i >= end; i --)
    {
        if (isl_is_flat(Left_Line, i))
        {
            continue;                           /* 一段平坦, 不能作为转折点 */
        }
        saw_slope = 1;
        if (isl_is_local_max(Left_Line, i, prom))
        {
            ret = i;
            break;
        }
    }

    /* 诊断原因码, 见 Isl_Mono_Why 定义处的说明 */
    if (ret > 0)         { Isl_Mono_Why = 5; }  /* 找到了 */
    else if (saw_slope)  { Isl_Mono_Why = 4; }  /* 有起伏但幅度不够 */
    else                 { Isl_Mono_Why = 3; }  /* 整段平坦 */

    return ret;
}

int Monotonicity_Change_Left(int start, int end)
{
    return isl_mono_change_left(start, end, ISL_TOP_MARGIN, ISL_MONO_PROM);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    单调性突变检测 —— 可指定顶部豁免行数的版本
  @param    top_skip  顶部豁免行数
  @return   突变点行号, 找不到返回 0
  @note     ★ 为什么需要这个版本 ★

            状态1/2 的补线要找的拐点本来就在第 5~15 行, 而 isl_clamp_range()
            会把 end 抬到  top = IMG_ROW - Search_Stop_Line + 2 + top_skip。

            用 ISL_TOP_MARGIN(=8) 时:
                SS=53(环岛入口实测值) -> top = 60-53+2+8 = 17
                17 > start(15)        -> top = 15
                end(5) < 15           -> end = 15
                start(15) <= end(15)  -> 直接 return 0
            要让它通过得 SS > 55, 而本车 SS 最大就是 55 ——
            也就是说状态1 的补线搜索用 8 是恒为 0 的, 补线一帧都画不出来。

            用 top_skip = 0 时:
                SS=53 -> top = 9, 扫描 15~9 行, 并且 isl_is_local_max 要访问的
                line[i-ISL_WIN] 最远到第 7 行, 正好在有效区内(有效区从第 7 行起),
                idx_margin 已经把 i±ISL_WIN 的余量算进去了, 不需要再多留。
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Left_Ex(int start, int end, int top_skip, int prom)
{
    return isl_mono_change_left(start, end, top_skip, prom);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    右边界单调性突变检�?  @return   突变点行�? 找不到返�?0
-------------------------------------------------------------------------------------------------------------------*/
static int isl_mono_change_right(int start, int end, int top_skip, int prom)
{
    int i;
    int ret = 0;
    int saw_slope = 0;

    Isl_Mono_Why = 0;

    if (Right_Lost_Time >= 0.9 * IMG_ROW)
    {
        Isl_Mono_Why = 1;
        return ret;
    }

    isl_clamp_range(&start, &end, ISL_WIN, top_skip);

    if (start <= end)
    {
        Isl_Mono_Why = 2;
        return ret;
    }

    for (i = start; i >= end; i --)
    {
        if (isl_is_flat(Right_Line, i))
        {
            continue;
        }
        saw_slope = 1;
        if (isl_is_local_min(Right_Line, i, prom))
        {
            ret = i;
            break;
        }
    }

    /* 诊断原因码, 同左边界版 */
    if (ret > 0)         { Isl_Mono_Why = 5; }
    else if (saw_slope)  { Isl_Mono_Why = 4; }
    else                 { Isl_Mono_Why = 3; }

    return ret;
}

int Monotonicity_Change_Right(int start, int end)
{
    return isl_mono_change_right(start, end, ISL_TOP_MARGIN, ISL_MONO_PROM);
}

/* 右边界版, top_skip 的含义和 Monotonicity_Change_Left_Ex 完全一样 */
int Monotonicity_Change_Right_Ex(int start, int end, int top_skip, int prom)
{
    return isl_mono_change_right(start, end, top_skip, prom);
}

/* ==========================================================================================
 *                                          四个角点
 * ========================================================================================== */

/*-------------------------------------------------------------------------------------------------------------------
  @brief    左下角点检�?  @return   角点所在行�? 找不到返�?0
  @note     作者注�? 下面5行数据不稳定, 不能作为边界点来判断, 舍弃
-------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Down_Point(int start, int end)
{
    int i;
    int ret = 0;

    if (Left_Lost_Time >= 0.9 * IMG_ROW)
    {
        return ret;
    }

    isl_clamp_range(&start, &end, 4, 0);           /* 判据里要访问 i±4 */

    if (end <= IMG_ROW - Search_Stop_Line)      /* 搜索截止行往上的全都不判 */
    {
        end = IMG_ROW - Search_Stop_Line;
    }
    if (end <= 0)
    {
        end = 0;
    }

    for (i = start; i >= end; i --)
    {
        if (abs(Left_Line[i]     - Left_Line[i + 1]) <= ISL_CORNER_STABLE &&
            abs(Left_Line[i + 1] - Left_Line[i + 2]) <= ISL_CORNER_STABLE &&
            abs(Left_Line[i + 2] - Left_Line[i + 3]) <= ISL_CORNER_STABLE &&
            (Left_Line[i] - Left_Line[i - 2]) >= ISL_CORNER_C2 &&
            (Left_Line[i] - Left_Line[i - 3]) >= ISL_CORNER_C3 &&
            (Left_Line[i] - Left_Line[i - 4]) >= ISL_CORNER_C4)
        {
            ret = i;
            break;
        }
    }
    return ret;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    左上角点检�?  @return   角点所在行�? 找不到返�?0
-------------------------------------------------------------------------------------------------------------------*/
static int isl_find_left_up(int start, int end, int stable, int c2, int c3, int c4)
{
    int i;
    int ret = 0;

    if (Left_Lost_Time >= 0.9 * IMG_ROW)
    {
        return ret;
    }

    isl_clamp_range(&start, &end, 4, 0);

    if (end <= IMG_ROW - Search_Stop_Line)
    {
        end = IMG_ROW - Search_Stop_Line;
    }
    if (end <= 0)
    {
        end = 0;
    }

    for (i = start; i >= end; i --)
    {
        if (abs(Left_Line[i]     - Left_Line[i - 1]) <= stable &&
            abs(Left_Line[i - 1] - Left_Line[i - 2]) <= stable &&
            abs(Left_Line[i - 2] - Left_Line[i - 3]) <= stable &&
            (Left_Line[i] - Left_Line[i + 2]) >= c2 &&
            (Left_Line[i] - Left_Line[i + 3]) >= c3 &&
            (Left_Line[i] - Left_Line[i + 4]) >= c4)
        {
            ret = i;
            break;
        }
    }
    return ret;
}

/* 用全局阈值 —— 状态0 的判据(下角点)走这条 */
int Find_Left_Up_Point(int start, int end)
{
    return isl_find_left_up(start, end,
                            ISL_CORNER_STABLE, ISL_CORNER_C2, ISL_CORNER_C3, ISL_CORNER_C4);
}

/* 自己指定阈值 —— 状态3 的补线走这条, 用的是 ISL3_S3_UP_* (比全局松)。
   为什么必须分开: ISL_CORNER_* 是给状态0 的【下角点】调稳的(直道上不能乱报),
   全局放松会连带把入环判据弄松。原因详见 ISL3_S3_UP_* 的说明。 */
int Find_Left_Up_Point_Ex(int start, int end, int stable, int c2, int c3, int c4)
{
    return isl_find_left_up(start, end, stable, c2, c3, c4);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    右下角点检�?  @return   角点所在行�? 找不到返�?0
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Down_Point(int start, int end)
{
    int i;
    int ret = 0;

    if (Right_Lost_Time >= 0.9 * IMG_ROW)
    {
        return ret;
    }

    isl_clamp_range(&start, &end, 4, 0);

    if (end <= IMG_ROW - Search_Stop_Line)
    {
        end = IMG_ROW - Search_Stop_Line;
    }
    if (end <= 0)
    {
        end = 0;
    }

    for (i = start; i >= end; i --)
    {
        if (abs(Right_Line[i]     - Right_Line[i + 1]) <= ISL_CORNER_STABLE &&
            abs(Right_Line[i + 1] - Right_Line[i + 2]) <= ISL_CORNER_STABLE &&
            abs(Right_Line[i + 2] - Right_Line[i + 3]) <= ISL_CORNER_STABLE &&
            (Right_Line[i] - Right_Line[i - 2]) <= -ISL_CORNER_C2 &&
            (Right_Line[i] - Right_Line[i - 3]) <= -ISL_CORNER_C3 &&
            (Right_Line[i] - Right_Line[i - 4]) <= -ISL_CORNER_C4)
        {
            ret = i;
            break;
        }
    }
    return ret;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    右上角点检�?  @return   角点所在行�? 找不到返�?0
-------------------------------------------------------------------------------------------------------------------*/
static int isl_find_right_up(int start, int end, int stable, int c2, int c3, int c4)
{
    int i;
    int ret = 0;

    if (Right_Lost_Time >= 0.9 * IMG_ROW)
    {
        return ret;
    }

    isl_clamp_range(&start, &end, 4, 0);

    if (end <= IMG_ROW - Search_Stop_Line)
    {
        end = IMG_ROW - Search_Stop_Line;
    }
    if (end <= 0)
    {
        end = 0;
    }

    for (i = start; i >= end; i --)
    {
        if (abs(Right_Line[i]     - Right_Line[i - 1]) <= stable &&
            abs(Right_Line[i - 1] - Right_Line[i - 2]) <= stable &&
            abs(Right_Line[i - 2] - Right_Line[i - 3]) <= stable &&
            (Right_Line[i] - Right_Line[i + 2]) <= -c2 &&
            (Right_Line[i] - Right_Line[i + 3]) <= -c3 &&
            (Right_Line[i] - Right_Line[i + 4]) <= -c4)
        {
            ret = i;
            break;
        }
    }
    return ret;
}

int Find_Right_Up_Point(int start, int end)
{
    return isl_find_right_up(start, end,
                             ISL_CORNER_STABLE, ISL_CORNER_C2, ISL_CORNER_C3, ISL_CORNER_C4);
}

/* 自己指定阈值, 同 Find_Left_Up_Point_Ex */
int Find_Right_Up_Point_Ex(int start, int end, int stable, int c2, int c3, int c4)
{
    return isl_find_right_up(start, end, stable, c2, c3, c4);
}

/* ==========================================================================================
 *                                          补线
 * ========================================================================================== */

/*-------------------------------------------------------------------------------------------------------------------
  @brief    在二值图上画一条黑�?  @param    (x1,y1) 起点, (x2,y2) 终点   注意 x 是列, y 是行
  @note     原作者工程里�?Draw_Line, 本工程原本没�? 这里补上�?            画进 BinaryImg_CDM, 因为 Longest_White_Column() 扫的就是这张图�?            �?Bresenham, 不会断线也不会重复画�?-------------------------------------------------------------------------------------------------------------------*/
void Draw_Line(int x1, int y1, int x2, int y2)
{
    int dx, dy, sx, sy, err, e2;

    if (x1 < 0)             { x1 = 0; }
    if (x1 > IMG_COL - 1)   { x1 = IMG_COL - 1; }
    if (x2 < 0)             { x2 = 0; }
    if (x2 > IMG_COL - 1)   { x2 = IMG_COL - 1; }
    if (y1 < 0)             { y1 = 0; }
    if (y1 > IMG_ROW - 1)   { y1 = IMG_ROW - 1; }
    if (y2 < 0)             { y2 = 0; }
    if (y2 > IMG_ROW - 1)   { y2 = IMG_ROW - 1; }

    dx =  abs(x2 - x1);
    dy = -abs(y2 - y1);
    sx = (x1 < x2) ? 1 : -1;
    sy = (y1 < y2) ? 1 : -1;
    err = dx + dy;

    for (;;)
    {
        BinaryImg_CDM[y1][x1] = 0;              /* 0 = �?*/

        if ((x1 == x2) && (y1 == y2))
        {
            break;
        }

        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x1 += sx; }
        if (e2 <= dx) { err += dx; y1 += sy; }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    状�? 的半圆补�?  @param    anchor_row / anchor_col  弧顶(状�? 找到的那个角�?
            dir                       半圆往哪边�? -1 往�? +1 往�?  @note     以角点为圆心, 画一个半�?R 的半圆到二值图�?黑色), 画完必须重新调用
            Longest_White_Column() 才会生效�?
            参数方程: x = anchor_col + dir * sqrt(R^2 - dy^2), dy �?-R 扫到 +R
            这样弧从 (col, row-R) 出发, 经过 (col ± R, row), 收在 (col, row+R),
            是一个完整的半圆�?
            为什么不用直�? 环岛是圆�? 直线补出来是切线, 车会沿直线冲出去, 拐不进环�?-------------------------------------------------------------------------------------------------------------------*/
static void Island_Draw_Arc(int anchor_row, int anchor_col, int dir)
{
    float r;
    int   i, i0, i1;
    int   last_x = 0, last_y = 0;
    uint8 first = 1;

    if ((anchor_row < 0) || (anchor_row >= IMG_ROW))
    {
        return;
    }

    /* ---- 半径: 用弧顶那一行的标准赛宽�?---- */
    if (Base_Width[anchor_row] > 0)
    {
        r = (float)Base_Width[anchor_row] * ISL_ARC_R_RATIO;
    }
    else
    {
        r = (float)IMG_COL * 0.25f;         /* 那一行没标定出来, 退一个保守�?*/
    }

    if (r < ISL_ARC_R_MIN) { r = ISL_ARC_R_MIN; }
    if (r > ISL_ARC_R_MAX) { r = ISL_ARC_R_MAX; }

    /* ---- 逐行描弧 ---- */
    i0 = -(int)r;
    i1 =  (int)r;

    for (i = i0; i <= i1; i ++)
    {
        float dy  = (float)i;
        float dx2 = r * r - dy * dy;
        int   x, y;

        if (dx2 < 0.0f) { dx2 = 0.0f; }

        x = anchor_col + dir * (int)(sqrtf(dx2) + 0.5f);
        y = anchor_row + i;

        if ((y < 0) || (y >= IMG_ROW))
        {
            continue;                       /* 超出图像的行直接跳过 */
        }

        if (first)
        {
            last_x = x;
            last_y = y;
            first  = 0;
            continue;                       /* 第一个点只记下来, 还没有线段可�?*/
        }

        Draw_Line(last_x, last_y, x, y);    /* 相邻两点之间连一小段 */
        last_x = x;
        last_y = y;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    按斜率拉一条直�? 补成左边�?  @param    k       斜率
            startX  起始点列坐标
            startY  起始点行坐标
            endY    结束点行坐标
  @note     作者注�? 补得线直接贴在边线上�?(y-y1)=k(x-x1) 变形 -> x=(y-y1)/k+x1
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Left(float k, int startX, int startY, int endY)
{
    int i, t;

    if (startY >= IMG_ROW - 1)  { startY = IMG_ROW - 1; }
    else if (startY <= 0)       { startY = 0; }
    if (endY >= IMG_ROW - 1)    { endY = IMG_ROW - 1; }
    else if (endY <= 0)         { endY = 0; }

    if (startY < endY)                          /* -- 操作, start 需要大 */
    {
        t = startY;
        startY = endY;
        endY = t;
    }

    if (k == 0.0f)                              /* 除零保护, 原代码没�?*/
    {
        return;
    }

    /* ★ 标记这几行"补过了" ★
       和 Left_Add_Line 里那一句同理: 下面写的 Left_Line[] 会被 Left_Lost_Flag
       那道闸门挡住 —— Calculate_Mid_Line 和 TFT180_Show_Track_Lines 都看那个标志,
       不打标记就是白补。状态4/5/6 的出环补线走的就是这里。 */
    Isl_Patch_Mark(startY, endY);

    for (i = startY; i >= endY; i --)
    {
        Left_Line[i] = (int)((i - startY) / k + startX);

        if (Left_Line[i] >= IMG_COL - 1)    { Left_Line[i] = IMG_COL - 1; }
        else if (Left_Line[i] <= 0)         { Left_Line[i] = 0; }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    按斜率拉一条直�? 补成右边�?  @note     参数含义�?K_Add_Boundry_Left
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Right(float k, int startX, int startY, int endY)
{
    int i, t;

    if (startY >= IMG_ROW - 1)  { startY = IMG_ROW - 1; }
    else if (startY <= 0)       { startY = 0; }
    if (endY >= IMG_ROW - 1)    { endY = IMG_ROW - 1; }
    else if (endY <= 0)         { endY = 0; }

    if (startY < endY)
    {
        t = startY;
        startY = endY;
        endY = t;
    }

    if (k == 0.0f)
    {
        return;
    }

    /* 同 K_Add_Boundry_Left: 不标记的话补线会被丢线标志挡住, 等于白补 */
    Isl_Patch_Mark(startY, endY);

    for (i = startY; i >= endY; i --)
    {
        Right_Line[i] = (int)((i - startY) / k + startX);

        if (Right_Line[i] >= IMG_COL - 1)   { Right_Line[i] = IMG_COL - 1; }
        else if (Right_Line[i] <= 0)        { Right_Line[i] = 0; }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    按斜率在二值图上画一条黑�?  @note     补完要重新调�?Longest_White_Column() 刷新边界数据
-------------------------------------------------------------------------------------------------------------------*/
void K_Draw_Line(float k, int startX, int startY, int endY)
{
    int endX = 0;

    if (startX >= IMG_COL - 1)  { startX = IMG_COL - 1; }
    else if (startX <= 0)       { startX = 0; }
    if (startY >= IMG_ROW - 1)  { startY = IMG_ROW - 1; }
    else if (startY <= 0)       { startY = 0; }
    if (endY >= IMG_ROW - 1)    { endY = IMG_ROW - 1; }
    else if (endY <= 0)         { endY = 0; }

    if (k == 0.0f)                              /* 除零保护, 原代码没�?*/
    {
        return;
    }

    endX = (int)((endY - startY) / k + startX);
    Draw_Line(startX, startY, endX, endY);
}

/* ==========================================================================================
 *                                          工具
 * ========================================================================================== */

/*-------------------------------------------------------------------------------------------------------------------
  @brief    取某几行的平均赛�?  @param    start_line, end_line  起始�?/ 中止�?-------------------------------------------------------------------------------------------------------------------*/
int Get_Road_Wide(int start_line, int end_line)
{
    int i, t;
    int road_wide = 0;

    if (start_line >= IMG_ROW - 1)  { start_line = IMG_ROW - 1; }
    else if (start_line <= 0)       { start_line = 0; }
    if (end_line >= IMG_ROW - 1)    { end_line = IMG_ROW - 1; }
    else if (end_line <= 0)         { end_line = 0; }

    if (start_line > end_line)                  /* ++ 访问, 坐标反了互换 */
    {
        t = start_line;
        start_line = end_line;
        end_line = t;
    }

    for (i = start_line; i <= end_line; i ++)
    {
        road_wide += Right_Line[i] - Left_Line[i];
    }

    return road_wide / (end_line - start_line + 1);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    取左边界在两点之间的斜率
-------------------------------------------------------------------------------------------------------------------*/
float Get_Left_K(int start_line, int end_line)
{
    int t;

    if (start_line >= IMG_ROW - 1)  { start_line = IMG_ROW - 1; }
    else if (start_line <= 0)       { start_line = 0; }
    if (end_line >= IMG_ROW - 1)    { end_line = IMG_ROW - 1; }
    else if (end_line <= 0)         { end_line = 0; }

    if (start_line > end_line)
    {
        t = start_line;
        start_line = end_line;
        end_line = t;
    }

    return (float)((float)Left_Line[start_line] - (float)Left_Line[end_line])
           / (float)(end_line - start_line + 1);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    取右边界在两点之间的斜率
-------------------------------------------------------------------------------------------------------------------*/
float Get_Right_K(int start_line, int end_line)
{
    int t;

    if (start_line >= IMG_ROW - 1)  { start_line = IMG_ROW - 1; }
    else if (start_line <= 0)       { start_line = 0; }
    if (end_line >= IMG_ROW - 1)    { end_line = IMG_ROW - 1; }
    else if (end_line <= 0)         { end_line = 0; }

    if (start_line > end_line)
    {
        t = start_line;
        start_line = end_line;
        end_line = t;
    }

    return (float)((float)Right_Line[start_line] - (float)Right_Line[end_line])
           / (float)(end_line - start_line + 1);
}

/* ==========================================================================================
 *                                    偏航�?(替代陀螺仪)
 * ========================================================================================== */

/*-------------------------------------------------------------------------------------------------------------------
  @brief    偏航角更�? �?TIM6 �?10ms 调用一�?  @param    speed_l / speed_r  左右轮当前速度(编码器计�?/ 10ms)
  @note     原作者这里接的是陀螺仪 Z 轴角速度积分。本工程没有陀螺仪,
             用左右轮速差死推: 角速度 �?(v�?- v�? / 轮距�?             所有单位换�?轮径、每圈计数、轮距�?0ms 周期、弧度转角度)都折�?             Island_Yaw_Gain 一个系数里, 实车标定即可�?-------------------------------------------------------------------------------------------------------------------*/
void Island_Yaw_Update(int speed_l, int speed_r)
{
    FJ_Angle += (float)(speed_r - speed_l) * Island_Yaw_Gain;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    环岛任务复位, 发车(GO �?0 �?1)时调�?-------------------------------------------------------------------------------------------------------------------*/
void Island_Reset(void)
{
    Island_State      = 0;
    Left_Island_Flag  = 0;
    Right_Island_Flag = 0;
    Left_Up_Guai[0]   = 0;
    Left_Up_Guai[1]   = 0;
    Right_Up_Guai[0]  = 0;
    Right_Up_Guai[1]  = 0;
    Island_Over_Flag  = 0;
    FJ_Angle          = 0.0f;

    /* 状态1 诊断快照一起清零, 这样每次发车看到的都是这一趟的数据 */
    Isl_Diag_S1_L   = 0;
    Isl_Diag_S1_R   = 0;
    Isl_Diag_S1_DUR = 0;
    Isl_Diag_S1_SS  = 0;
    Isl_Diag_S1_TOP = 0;
    Isl_Diag_S1_LR  = 0;
    Isl_Diag_S1_BS  = 0;
    Isl_Diag_S1_LT  = 0;
    Isl_Diag_S1_WHY = 0;
    Isl_Diag_S1_HIT = 0;
    {
        int b;
        for (b = 0; b < ISL3_BAND_N; b ++) { Isl_Diag_S1_BAND[b] = 0; }
    }

    island_arc_on  = 0;
    island_arc_row = 0;
    island_arc_col = 0;
}

/* ==========================================================================================
 *                                          环岛检�? *
 *  状态含�?(原作者定�?
 *      0  没看到环�? *      1  刚认定是环岛, 下拐点还�? *      2  下拐点丢�? 上方开始出现弧�? *      3  准备进环, 找上拐点并拉死线
 *      4  已经在环岛里�? *      5  准备出环
 *      6  继续出环
 *      7  基本出环, 找拐�? *      8  连线, 出环最后一�? *      9  环岛结束(防止连续判环, 下一帧由外部归零)
 * ========================================================================================== */
void Island_Detect(void)
{
    static float k = 0.0f;                      /* 状�?�?用的斜率 */
    static int   island_state_5_down[2] = {0};  /* 状�?: 即将离开环岛时边界的最低点 */
    static int   left_down_guai[2]      = {0};  /* 认定环岛时用的左下角�?*/
    static int   right_down_guai[2]     = {0};  /* 认定环岛时用的右下角�?*/
    static uint8 isl_left_hit           = 0;    /* 左环岛判据连续成立的帧数(防斜入十字误�? */
    static uint8 isl_right_hit          = 0;    /* 右环岛判据连续成立的帧数 */

    int monotonicity_change_line[2];            /* [0] �? [1] �?*/
    int monotonicity_change_left_flag  = 0;
    int monotonicity_change_right_flag = 0;
    int continuity_change_left_flag    = 0;
    int continuity_change_right_flag   = 0;

    /* ---- 每帧先算四个基础判据 ---- */
    continuity_change_left_flag    = Continuity_Change_Left (IMG_ROW - 1 - 2, 5);   /* �?(H-1-5, 10) */
    continuity_change_right_flag   = Continuity_Change_Right(IMG_ROW - 1 - 2, 5);
    monotonicity_change_right_flag = Monotonicity_Change_Right(IMG_ROW - 1 - 5, 5); /* �?(H-1-10, 10) */
    monotonicity_change_left_flag  = Monotonicity_Change_Left (IMG_ROW - 1 - 5, 5);

    /* 给菜单显示用 */
    Island_Dbg_CL = continuity_change_left_flag;
    Island_Dbg_CR = continuity_change_right_flag;
    Island_Dbg_ML = monotonicity_change_left_flag;
    Island_Dbg_MR = monotonicity_change_right_flag;

    /* ---- 状�?9 �?刚跑�?, 由这里归�? 防止连续判环 ---- */
    if (Island_State == 9)
    {
        Island_State = 0;
        Island_Over_Flag = 1;
    }

    /* ==================================================================================
     *  �?0 �? 认定环岛 (只在完全没状态、没十字、没坡道的时候判)
     * ================================================================================== */
    /* �?刷屏优化: 这里原来又把 Continuity_Change_Left/Right 算了一�?
       参数和上面第 1040~1041 行一模一�? 而且这中间没有改�?Left_Line/Right_Line,
       结果完全相同 —�?纯重复计�? 直接删掉复用上面的�?*/
    if (Cross_Flag == 0 && Island_State == 0 && Ramp_Flag == 0)
    {
        /* ---------------- 左环�?---------------- */
        if (Left_Island_Flag == 0)
        {
            if (monotonicity_change_right_flag == 0      &&   /* 右边是单调的 */
                continuity_change_left_flag    != 0      &&   /* 左边是不连续�?*/
                continuity_change_right_flag   == 0      &&   /* 左环岛右边是连续�?*/
                Left_Lost_Time  >= ISL_LOST_MIN          &&   /* 左边丢线很多 */
                Left_Lost_Time  <= ISL_LOST_MAX          &&   /* 也不能全丢了 */
                Right_Lost_Time <= ISL_LOST_OTHER        &&   /* 右边丢线较少 */
                Search_Stop_Line >= ISL_STOP_MIN       &&   /* 搜索截止行看到很�?*/
                Boundry_Start_Left  >= ISL_START_NEAR    &&   /* 边界起始点靠�?*/
                Boundry_Start_Right >= ISL_START_NEAR    &&
                Both_Lost_Time <= ISL_LOST_OTHER)             /* 双边丢线�?*/
            {
                left_down_guai[0] = Find_Left_Down_Point(IMG_ROW - 1, 10);  /* �?(H-1, 20) */

                if (left_down_guai[0] >= ISL_DOWN_GONE)     /* �?>= 30 */
                {
                    /* �?连续 Isl_Confirm_Frames 帧都成立才真的认�?防斜入十字误�? �?*/
                    if (isl_left_hit < Isl_Confirm_Frames)
                    {
                        isl_left_hit ++;
                    }

                    if (isl_left_hit >= Isl_Confirm_Frames)
                    {
                        Island_State     = 1;
                        Left_Island_Flag = 1;
                        FJ_Angle         = 0.0f;    /* �?入环开始积�?—�?原作者靠陀螺仪, 这里必须清零 */
                        isl_left_hit     = 0;
                    }
                }
                else                                        /* 误判, 归零 */
                {
                    isl_left_hit     = 0;
                    Island_State     = 0;
                    Left_Island_Flag = 0;
                }
            }
            else
            {
                isl_left_hit = 0;       /* 判据断了就重新数 */
            }
        }

        /* ---------------- 右环�?---------------- */
        if (Right_Island_Flag == 0)
        {
            if (monotonicity_change_left_flag == 0     &&
                continuity_change_left_flag   == 0     &&   /* 右环岛左边是连续�?*/
                continuity_change_right_flag  != 1     &&   /* 右边是不连续�?*/
                Right_Lost_Time >= ISL_LOST_MIN        &&
                Right_Lost_Time <= ISL_LOST_MAX        &&
                Left_Lost_Time  <= ISL_LOST_OTHER      &&
                Search_Stop_Line >= ISL_STOP_MIN     &&
                Boundry_Start_Left  >= ISL_START_NEAR  &&
                Boundry_Start_Right >= ISL_START_NEAR  &&
                Both_Lost_Time <= ISL_LOST_OTHER)
            {
                right_down_guai[0] = Find_Right_Down_Point(IMG_ROW - 1, 10);

                if (right_down_guai[0] >= ISL_DOWN_GONE)
                {
                    /* �?同左环岛: 连续 Isl_Confirm_Frames 帧都成立才认�?�?*/
                    if (isl_right_hit < Isl_Confirm_Frames)
                    {
                        isl_right_hit ++;
                    }

                    if (isl_right_hit >= Isl_Confirm_Frames)
                    {
                        Island_State      = 1;
                        Right_Island_Flag = 1;
                        FJ_Angle          = 0.0f;   /* �?入环开始积�?*/
                        isl_right_hit     = 0;
                    }
                }
                else
                {
                    isl_right_hit     = 0;
                    Island_State      = 0;
                    Right_Island_Flag = 0;
                }
            }
            else
            {
                isl_right_hit = 0;      /* 判据断了就重新数 */
            }
        }
    }
    else if (Cross_Flag != 0)
    {
        /* �?十字期间必须把确认计数清�?�?           否则斜入十字�?Cross_Flag 可能�?0/1 之间来回�?
           计数会在那些"恰好�?0"的帧上偷偷累�? 防抖就白做了�?*/
        isl_left_hit  = 0;
        isl_right_hit = 0;
    }

    /* ==================================================================================
     *  左环岛状态机
     * ================================================================================== */
    if (Left_Island_Flag == 1)
    {
        /* ---- 状�?: 拐点还在, 没丢�?---- */
        if (Island_State == 1)
        {
            /* �?先判角点在不�?—�?必须�?补线之前"的原始边�?�?*/
            uint8 down_gone = (uint8)(Find_Left_Down_Point(IMG_ROW - 1, 10) == 0);

            monotonicity_change_line[0] = Monotonicity_Change_Left(ISL_DOWN_SEARCH, ISL_WIN); /* �?(30,5) */

            /* 找到转折点才补线, 否则会把整条左边界拉�?(原因见右环岛那边的注�? */
            if (monotonicity_change_line[0] > 0)
            {
                monotonicity_change_line[1] = Left_Line[monotonicity_change_line[0]];
                /* �?必须�?0.15f 不能�?0.15 �?                   写成 0.15 (double 字面�? 的话, 整个表达式会被提升成 double:
                   int->double(__floatsidf) + 双精度乘�?__muldf3) + double->int(__fixdfsi),
                   三个都是软件模拟函数(本芯片只有单精度 FPU), 一次几百个周期�?                   加上后缀 f 之后变成一�?FPU 乘法指令 + 一�?fcvt, 结果完全一�?                   (反正最后都要截断成整数)�?*/
                Left_Add_Line((int)(monotonicity_change_line[1] * 0.15f), IMG_ROW - 1,
                              monotonicity_change_line[1], monotonicity_change_line[0]);
            }

            /* �?状�? -> 2: 左下角点消失 / 下方丢线 �?               两条判据并联, 任一条成立就�?2:
                 - 角点本身找不到了 (Find_Left_Down_Point 返回 0)
                 - Boundry_Start_Left 掉到 ISL_DOWN_BS 以下 (下方一片丢�?
               注意必须用补线之前的原始边界, 所�?down_gone 在上面就算好了�?*/
            if (down_gone || (Boundry_Start_Left <= ISL_DOWN_BS))
            {
                Island_State = 2;
            }
        }

        /* ---- 状�?: 下方角点消失, 上面是弧�?---- */
        else if (Island_State == 2)
        {
            monotonicity_change_line[0] = Monotonicity_Change_Left(ISL_ARC_SEARCH, 5);  /* �?(70,10) */

            if (monotonicity_change_line[0] > 0)    /* 找不到转折点就不�? 否则会把边界拉直 */
            {
                monotonicity_change_line[1] = Left_Line[monotonicity_change_line[0]];
                Left_Add_Line((int)(monotonicity_change_line[1] * 0.1f), IMG_ROW - 1,   /* 0.1f 见上�?0.15f 的注�?*/
                              monotonicity_change_line[1], monotonicity_change_line[0]);
            }

            if ((Boundry_Start_Left >= IMG_ROW - 2) ||  /* �?H-5 */
                (monotonicity_change_line[0] > ISL_ARC_LOW))    /* �?> 50, 圆弧靠下了进3 */
            {
                Island_State     = 3;
                Left_Island_Flag = 1;
            }
        }

        /* ---- 状�?: 准备进环, 找上拐点并画半圆 ---- */
        else if (Island_State == 3)
        {
            if (island_arc_on)
            {
                /* �?刷屏优化: 重扫边界只有�?真的往图上画了�?之后才有意义�?                   关掉半圆补线(ISL_ARC_ENABLE==0)时原来这里每帧白扫一整幅�?
                   纯浪�? 所以把重扫和画弧绑在同一个开关里 */
                if (ISL_ARC_ENABLE)
                {
                    Island_Draw_Arc(island_arc_row, island_arc_col, ISL_ARC_DIR_L);
                    Longest_White_Column();                     /* 弧画进图里了, 必须重扫才生�?*/
                }
            }
            else
            {
                Left_Up_Guai[0] = Find_Left_Up_Point(ISL_UP_SEARCH_HI, ISL_UP_SEARCH_LO); /* �?(40,5) */
                Left_Up_Guai[1] = Left_Line[Left_Up_Guai[0]];

                if (Left_Up_Guai[0] < ISL_UP_BAIL)          /* �?< 5, 防止误判 */
                {
                    Island_State     = 0;
                    Left_Island_Flag = 0;
                }
                else if (Left_Up_Guai[0] > 0)
                {
                    /* 角点找到就画半圆 (原因见右环岛那边的注�? */
                    island_arc_row = Left_Up_Guai[0];
                    island_arc_col = Left_Up_Guai[1];
                    island_arc_on  = 1;

                    if (ISL_ARC_ENABLE)
                    {
                        Island_Draw_Arc(island_arc_row, island_arc_col, ISL_ARC_DIR_L);
                        Longest_White_Column();
                    }
                }
            }

            /* �?原作者此处靠陀螺仪积分判断"已经开进环岛了"�?               本工�?FJ_Angle 由左右轮速差积分得到, 阈�?60 度沿�?*/
            if (abs((int)FJ_Angle) >= (int)ISL_ANGLE_IN)
            {
                island_arc_on = 0;
                k = 0.0f;
                Island_State = 4;
                /* 这里原来也重扫了一�? 但这一支什么都没往图上�? 扫了也是白扫, 去掉 */
            }
        }

        /* ---- 状�?: 已经在环岛里�?---- */
        else if (Island_State == 4)
        {
            if (abs((int)FJ_Angle) > (int)ISL_ANGLE_MID)    /* 积分 200 度后才打开出环判断 */
            {
                monotonicity_change_line[0] = Monotonicity_Change_Right(IMG_ROW - 5, 5); /* �?(H-10,10) */
                monotonicity_change_line[1] = Right_Line[monotonicity_change_line[0]];

                if ((monotonicity_change_line[0] >= ISL_MONO_ROW_MIN) &&
                    (monotonicity_change_line[0] <= ISL_MONO_ROW_MAX) &&     /* �?35~55 */
                    (monotonicity_change_line[1] >= ISL_MONO_COL_MIN))       /* �?>= 10 */
                {
                    island_state_5_down[0] = IMG_ROW - 1;
                    island_state_5_down[1] = Right_Line[IMG_ROW - 1];

                    if (island_state_5_down[1] != monotonicity_change_line[1])   /* 除零保护 */
                    {
                        k = (float)(IMG_ROW - monotonicity_change_line[0])
                            / (float)(island_state_5_down[1] - monotonicity_change_line[1]);

                        K_Add_Boundry_Right(k, island_state_5_down[1], island_state_5_down[0], 0);
                        Island_State = 5;
                    }
                }
            }
        }

        /* ---- 状�?: 出环 ---- */
        else if (Island_State == 5)
        {
            K_Add_Boundry_Right(k, island_state_5_down[1], island_state_5_down[0], 0);

            if (Boundry_Start_Right < ISL_START_NEAR)       /* �?< H-20, 右边先丢�?*/
            {
                Island_State = 6;
            }
        }

        /* ---- 状�?: 还在�?---- */
        else if (Island_State == 6)
        {
            K_Add_Boundry_Right(k, island_state_5_down[1], island_state_5_down[0], 0);

            if ((Boundry_Start_Right > ISL_START_VERY_NEAR) ||      /* �?> H-10, 右边不丢�?*/
                (abs((int)FJ_Angle) >= (int)ISL_ANGLE_OUT))         /* 或者角度够�?*/
            {
                k = 0.0f;
                Island_State = 7;
            }
        }

        /* ---- 状�?: 基本出去�? 找拐�?---- */
        else if (Island_State == 7)
        {
            Left_Up_Guai[0] = Find_Left_Up_Point(ISL_EXIT_SEARCH, 0);   /* �?(H-10, 0) */
            Left_Up_Guai[1] = Left_Line[Left_Up_Guai[0]];

            if ((Left_Up_Guai[1] <= ISL_EXIT_COL_L) &&                  /* �?<= 100 */
                (Left_Up_Guai[0] >= ISL_EXIT_ROW_MIN) &&
                (Left_Up_Guai[0] <= ISL_EXIT_ROW_MAX))                  /* �?5 ~ H-20 */
            {
                Island_State = 8;
            }
        }

        /* ---- 状�?: 连线, 出环最后一�?---- */
        else if (Island_State == 8)
        {
            Left_Up_Guai[0] = Find_Left_Up_Point(IMG_ROW - 1, ISL_EXIT_SEARCH_LO);  /* �?(H-1, 10) */
            Left_Up_Guai[1] = Left_Line[Left_Up_Guai[0]];
            Lengthen_Left_Boundry(Left_Up_Guai[0] - 1, IMG_ROW - 1);

            /* 拐点靠下, 或者拐点丢了但下方不丢�?-> 认为出环�?*/
            if ((Left_Up_Guai[0] >= ISL_EXIT_ROW_MAX) ||                /* �?>= H-20 */
                ((Left_Up_Guai[0] < ISL_EXIT_LOST_ROW) &&               /* �?< 10 */
                 (Boundry_Start_Left >= ISL_START_VERY_NEAR)))          /* �?>= H-10 */
            {
                FJ_Angle         = 0.0f;
                Island_State     = 9;
                Left_Island_Flag = 0;
            }
        }
    }

    /* ==================================================================================
     *  右环岛状态机
     * ================================================================================== */
    else if (Right_Island_Flag == 1)
    {
        /* ---- 状�? ---- */
        if (Island_State == 1)
        {
            /* �?先判角点在不�?—�?必须�?补线之前"的原始边�?
               下面�?Right_Add_Line 会改�?Right_Line, 之后再找角点找的就是假数据了 �?*/
            uint8 down_gone = (uint8)(Find_Right_Down_Point(IMG_ROW - 1, 10) == 0);

            /* 半边补线: 从环岛口拉一条到图像底部 */
            monotonicity_change_line[0] = Monotonicity_Change_Right(ISL_DOWN_SEARCH, ISL_WIN);

            /* �?找不到转折点(mcl[0]==0)就绝对不要补�?�?               因为下面要访�?Right_Line[mcl[0]] = Right_Line[0], 而且 y2=0 会让
               Right_Add_Line 把整条右边界从第 0 行到�?59 行全部改写成一条直�?
               边界被毁掉之后角点判据必然返�?0, 状态机会一路冲到状�?3�?*/
            if (monotonicity_change_line[0] > 0)
            {
                monotonicity_change_line[1] = Right_Line[monotonicity_change_line[0]];
                Right_Add_Line((int)(IMG_COL - 1 - (monotonicity_change_line[1] * 0.15f)), IMG_ROW - 1,  /* 0.15f 见状�?左环那条的注�?*/
                               monotonicity_change_line[1], monotonicity_change_line[0]);
            }

            /* �?状�? -> 2: 右下角点消失 / 下方丢线 �?(左环那条的镜�? 原因见那边注�?
               实测: 车停在环岛口 BS=59; 往前开、角点消失的一瞬间 BS 掉到 28,
                     所�?ISL_DOWN_BS �?30 正好卡在这个跳变上�?*/
            if (down_gone || (Boundry_Start_Right <= ISL_DOWN_BS))
            {
                Island_State = 2;
            }
        }

        /* ---- 状�?: 下方丢线, 上方即将出现大弧�?---- */
        else if (Island_State == 2)
        {
            monotonicity_change_line[0] = Monotonicity_Change_Right(ISL_ARC_SEARCH, ISL_WIN);  /* �?(70,5) */

            if (monotonicity_change_line[0] > 0)    /* 找不到转折点就不�? 否则会把边界拉直 */
            {
                monotonicity_change_line[1] = Right_Line[monotonicity_change_line[0]];
                Right_Add_Line((int)(IMG_COL - 1 - (monotonicity_change_line[1] * 0.15f)), IMG_ROW - 1,
                               monotonicity_change_line[1], monotonicity_change_line[0]);
            }

            if ((Boundry_Start_Right >= IMG_ROW - 2) ||     /* �?H-5 */
                (monotonicity_change_line[0] > ISL_ARC_LOW))/* �?> 50 */
            {
                Island_State      = 3;
                Right_Island_Flag = 1;
            }
        }

        /* ---- 状�?: 下面出现大弧�? 上方出现角点 ---- */
        else if (Island_State == 3)
        {
            if (island_arc_on)
            {
                /* 半圆已经锁定, 每帧重画一�?—�?二值图每帧重建, 上一帧画的弧不会留着�?                   重扫和画弧绑在同一个开关里, 关掉半圆补线就不用每帧白扫一幅图 */
                if (ISL_ARC_ENABLE)
                {
                    Island_Draw_Arc(island_arc_row, island_arc_col, ISL_ARC_DIR_R);
                    Longest_White_Column();
                }
            }
            else
            {
                Right_Up_Guai[0] = Find_Right_Up_Point(ISL_UP_SEARCH_HI, 5);    /* �?(40,10) */
                Right_Up_Guai[1] = Right_Line[Right_Up_Guai[0]];

                if (Right_Up_Guai[0] < ISL_UP_BAIL)             /* �?< 10, 防止误判 */
                {
                    Island_State      = 0;
                    Right_Island_Flag = 0;
                }
                else if (Right_Up_Guai[0] > 0)
                {
                    /* �?角点找到就画半圆 �?                       原来这里还有一�?角点必须落在 [7,25)x(25,55) 框里"的判�?
                       框不满足 k 就一直是 0、一条线都不�?—�?车当然不会拐�?                       现在半径�?Base_Width �? 角点位置本身不需要再限制�?*/
                    island_arc_row = Right_Up_Guai[0];
                    island_arc_col = Right_Up_Guai[1];
                    island_arc_on  = 1;

                    if (ISL_ARC_ENABLE)
                    {
                        Island_Draw_Arc(island_arc_row, island_arc_col, ISL_ARC_DIR_R);
                        Longest_White_Column();
                    }
                }
            }

            if (abs((int)FJ_Angle) >= (int)ISL_ANGLE_IN)        /* 只依靠角度积�?*/
            {
                island_arc_on = 0;
                k = 0.0f;
                Island_State = 4;
                /* 这一支没往图上画东�? 原来那次重扫是白�? 去掉 */
            }
        }

        /* ---- 状�?: 完全进去环岛�?---- */
        else if (Island_State == 4)
        {
            /* 注意: 原作者这里写的是 FJ_Angle>200 (带符�?, 而左环岛那边用的�?abs()�?               本工程偏航角是轮速差积分出来�? 正负号取决于编码器方向约�? 不好保证�?               所以这里也改成 abs(), 免得符号反了整个右环岛状态机卡在状�?不动�?*/
            if (abs((int)FJ_Angle) > (int)ISL_ANGLE_MID)        /* 积分 200 度后再打开单调转折判断 */
            {
                monotonicity_change_line[0] = Monotonicity_Change_Left(ISL_MONO_FAR, 5);  /* �?(90,10) */
                monotonicity_change_line[1] = Left_Line[monotonicity_change_line[0]];

                if ((monotonicity_change_line[0] >= ISL_MONO_ROW_MIN) &&
                    (monotonicity_change_line[0] <= ISL_MONO_ROW_MAX) &&
                    (monotonicity_change_line[1] < IMG_COL - 5))    /* �?< W-10 */
                {
                    island_state_5_down[0] = IMG_ROW - 1;
                    island_state_5_down[1] = Left_Line[IMG_ROW - 1] - 3;    /* �?-5, �?-3 */

                    if (island_state_5_down[1] != monotonicity_change_line[1])
                    {
                        k = (float)(IMG_ROW - monotonicity_change_line[0])
                            / (float)(island_state_5_down[1] - monotonicity_change_line[1]);

                        K_Add_Boundry_Left(k, island_state_5_down[1], island_state_5_down[0], 0);
                        Island_State = 5;
                    }
                }
            }
        }

        /* ---- 状�?: 准备出环�?---- */
        else if (Island_State == 5)
        {
            K_Add_Boundry_Left(k, island_state_5_down[1], island_state_5_down[0], 0);

            if (Boundry_Start_Left < ISL_START_NEAR)            /* 左边先丢�?*/
            {
                Island_State = 6;
            }
        }

        /* ---- 状�?: 继续�?---- */
        else if (Island_State == 6)
        {
            K_Add_Boundry_Left(k, island_state_5_down[1], island_state_5_down[0], 0);

            if ((Boundry_Start_Left > ISL_START_VERY_NEAR) ||
                (abs((int)FJ_Angle) >= (int)ISL_ANGLE_OUT))
            {
                k = 0.0f;
                Island_State = 7;
            }
        }

        /* ---- 状�?: 基本出环�? 找角�?---- */
        else if (Island_State == 7)
        {
            Right_Up_Guai[0] = Find_Right_Up_Point(ISL_EXIT_SEARCH, 0);     /* �?(H-10, 0) */
            Right_Up_Guai[1] = Right_Line[Right_Up_Guai[0]];

            if ((Right_Up_Guai[1] >= ISL_EXIT_COL_R) &&                     /* �?>= W-88 */
                (Right_Up_Guai[0] >= ISL_EXIT_ROW_MIN) &&
                (Right_Up_Guai[0] <= ISL_EXIT_ROW_MAX))
            {
                Island_State = 8;
            }
        }

        /* ---- 状�?: 环岛8 ---- */
        else if (Island_State == 8)
        {
            Right_Up_Guai[0] = Find_Right_Up_Point(IMG_ROW - 1, ISL_EXIT_SEARCH_LO);    /* �?(H-1, 10) */
            Right_Up_Guai[1] = Right_Line[Right_Up_Guai[0]];
            Lengthen_Right_Boundry(Right_Up_Guai[0] - 1, IMG_ROW - 1);

            if ((Right_Up_Guai[0] >= ISL_EXIT_ROW_MAX) ||
                ((Right_Up_Guai[0] < ISL_EXIT_LOST_ROW) &&
                 (Boundry_Start_Left >= ISL_START_VERY_NEAR)))
            {
                FJ_Angle          = 0.0f;
                Island_State      = 9;
                Right_Island_Flag = 0;
            }
        }
    }
}


/* ==========================================================================================
 *  ★ 环岛识别 —— 新版 (Island_Detect2) ★
 *
 *  这是给"右环岛 状态1/2, 以及 2->3 的判据"单独做的一套逻辑。
 *  ★ 原来的 Island_Detect() 一行都没动 ★
 *
 *  用法: main.c 里把   Island_Detect();   换成   Island_Detect2();   就行。
 *
 *  ------------------------------------------------------------------------------------
 *  和原版的区别:
 *
 *    原版 状态1/2:
 *        找到转折点后拉一条斜线(底部的列还乘了 0.15)把右边界掰过去。
 *
 *    新版 状态1/2 (两个状态做的是同一件事):
 *        把【图像右上的单调突变点】直接【向下连线】到图像底部, 当作新的右边界;
 *        中线照旧 = (左边界 + 右边界) / 2。
 *
 *    原版 2->3:
 *        (Boundry_Start_Right >= IMG_ROW-2) || (突变点行号 > ISL_ARC_LOW)
 *
 *    新版 2->3 (两条并联, 任一成立就进状态3):
 *        ① 单调突变点"纵坐标靠下"(行号 >= ISL2_MONO_ROW_LOW)
 *           或者"横坐标靠左"(列号 <= ISL2_MONO_COL_LEFT)
 *        ② 边界起始点靠下 (Boundry_Start_Right >= ISL2_BS_NEAR)
 *           —— 说明大圆弧已经来到视野下方
 *
 *  其余一切(状态0 的认定、左环岛、状态 4~8)原样交给 Island_Detect()。
 *  这样不用把 600 行状态机抄一遍, 而且以后改了原版, 新版这边自动跟着变。
 *  想把别的状态也换成新版, 把条件加进 Island_Detect2() 那个 if 就行。
 * ========================================================================================== */

/* ---- 新版 1/2/3 的判定参数 ---- */

/* 找"右上单调突变点"的搜索范围(行号): 从 ISL2_MONO_ROW_LO 往上扫到 ISL2_MONO_ROW_HI */
#define ISL2_MONO_ROW_LO      ISL_ARC_SEARCH    /* = 35, 上半部分的下沿 */
#define ISL2_MONO_ROW_HI      ISL_WIN           /* = 2,  接近图像顶部 */

/* 突变点"纵坐标靠下": 行号 >= 这个值就成立 */
#define ISL2_MONO_ROW_LOW     26
/* 突变点"横坐标靠左": 列号 <= 这个值就成立 (IMG_COL/2 = 47, 也就是图像中线) */
#define ISL2_MONO_COL_LEFT    (IMG_COL / 2)
/* 右边界起始点"靠下": >= 这个值就认为大圆弧下来了 */
#define ISL2_BS_NEAR          (IMG_ROW - 6)     /* = 54 */

/* 突变点列号小于这个值就当成噪声, 不补线。
   不保护的话右边界会被拉到图像最左边, 中线跟着跑到左边, 车会朝左猛打 —— 方向是反的。 */
#define ISL2_MONO_COL_MIN     6

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右环岛 状态1/2 的新逻辑 (状态1 和 状态2 做同一件事)
  @note      进来之前 Right_Island_Flag 已经是 1, 且 Island_State 是 1 或 2
-------------------------------------------------------------------------------------------------------------------*/
static void island_right_state12_new(void)
{
    int mono_row = 0;
    int mono_col = 0;

    /* ---- 找图像右上的单调突变点(右边界上的局部极小点) ---- */
    mono_row = Monotonicity_Change_Right(ISL2_MONO_ROW_LO, ISL2_MONO_ROW_HI);

    /* 给菜单显示用, 和原版的 MR 含义一致(就是 Monotonicity_Change_Right 的结果) */
    Island_Dbg_MR = mono_row;

    if (mono_row > 0)
    {
        /* ★ 必须在补线之前取列号 ★
           补线会改写 Right_Line[], 补完再取就读到假数据了。 */
        mono_col = Right_Line[mono_row];

        /* ---- 从突变点向下连线到图像底部, 作为新的右边界 ----
           这里连的是【垂直线】(底部的列 = 突变点的列)。
           想要斜线的话把最后一个参数 mono_col 改成别的列号即可。 */
        if (mono_col >= ISL2_MONO_COL_MIN)
        {
            Right_Add_Line(mono_col, mono_row, mono_col, IMG_ROW - 1);
        }
    }

    /* ======================= 状态1 ======================= */
    if (Island_State == 1)
    {
        /* 1 -> 2 的判据沿用原版, 没改:
             右下角点消失, 或者右边界起始点掉到 ISL_DOWN_BS 以下 */
        if ((Find_Right_Down_Point(IMG_ROW - 1, 10) == 0) ||
            (Boundry_Start_Right <= ISL_DOWN_BS))
        {
            Island_State = 2;
        }
        return;
    }

    /* ======================= 状态2 -> 3 =======================
       两条并联, 任一成立就进状态3 */
    if (((mono_row > 0) &&
         ((mono_row >= ISL2_MONO_ROW_LOW) ||        /* ① 突变点纵坐标靠下 */
          (mono_col <= ISL2_MONO_COL_LEFT))) ||      /*    或者横坐标靠左 */
        (Boundry_Start_Right >= ISL2_BS_NEAR))       /* ② 边界起始点靠下(大圆弧下来了) */
    {
        Island_State = 3;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     环岛识别(新版) —— 只接管右环岛的状态1/2, 其余全部委托给原版
  @note      main.c 里把 Island_Detect(); 换成 Island_Detect2(); 即可
-------------------------------------------------------------------------------------------------------------------*/
void Island_Detect2(void)
{
    /* 只有"右环岛 且 状态 1/2"由新逻辑接管。
       注意只接管 1/2: 状态3 及以后(找上拐点、角度积分、出环)全部走原版,
       这样两套状态机不会打架, 也不用把原版抄一遍。

       想把状态3 也接管过来的话, 把下面的条件改成
           (Island_State >= 1) && (Island_State <= 3)
       然后在 island_right_state12_new() 里加状态3 的分支就行。 */
    if ((Right_Island_Flag == 1) &&
        ((Island_State == 1) || (Island_State == 2)))
    {
        island_right_state12_new();
        return;
    }

    /* 其余一切照旧 */
    Island_Detect();
}


/* ==========================================================================================
 *  ★ 环岛识别 —— 版本3 (Island_Detect3) 原作者 188x120 版的完整移植 ★
 *
 *  来源: CSDN 那篇环岛文章的 Island_Detect(), 按 2:1 缩放到本工程的 94x60。
 *  原版 Island_Detect() 和我的 Island_Detect2() 都一行没动, 三个版本并存, 用哪个自己切。
 *
 *  ------------------------------------------------------------------------------------
 *  换算原则(和 island.c 顶部写的一样):
 *      行号 / 列号 / 行数 / 列数   ->  原值 / 2
 *      角度(60 / 200 / 320)       ->  不缩放
 *      比例(0.15 / 0.1 / 0.95)    ->  不缩放
 *      "离底部几行"(如 H-5)       ->  按比例缩(120 行的 5 行 ≈ 60 行的 2 行)
 *
 *  ★ 和原版唯一一处"故意的不同" ★
 *      原版没有在入环时清零 FJ_Angle —— 因为原作者用的是真陀螺仪。
 *      本工程没有陀螺仪, FJ_Angle 是左右轮速差从【发车】开始一路积下来的,
 *      不清零的话 60/200/320 三个角度阈值全部失去意义(可能一发车就满足)。
 *      所以下面在两处"认定环岛"的地方各加了一句 FJ_Angle = 0.0f。
 *
 *  ★ 第二处 ★
 *      原版右环岛状态4 写的是 FJ_Angle > 200 (带符号, 没取绝对值),
 *      左环岛那边却是 abs() —— 同一个作者两处不一致。
 *      本车偏航角的正负取决于编码器方向约定, 不好保证,
 *      所以统一改成 abs(), 免得符号反了右环岛卡在状态4出不来。
 *
 *  ★ 第三处 ★
 *      原版状态3 找不到上拐点时只清 Left/Right_Island_Flag,
 *      但同一帧后面的 abs(FJ_Angle)>=60 还能把 Island_State 推到 4。
 *      状态机被标志位关掉了却停在 4 —— 本工程 wheel.c 的 Island_Speed
 *      是看 Island_State>=2 触发的, 车会永远用环岛速度跑。
 *      所以两处都补了 return, 保证状态和标志位永远一致。
 *
 *  ★ 第四处 ★
 *      原版状态1/2 的补线没有判 "单调突变点是否存在"。
 *      Monotonicity_Change_* 返回 0 表示"没找到", 这时原版会去读 Left_Line[0]
 *      并画一条 y2=0 的补线 —— 那会把整条边界从第 0 行到第 59 行拉成一条直线。
 *      边界一毁, 后面所有角点判据全部返回 0, 状态机会一路冲到状态3。
 *      本工程原有的 Island_Detect 在同一位置就加了 if (mcl[0] > 0),
 *      这版照做, 四处补线全部加上。
 *
 *  ★ 第五处 —— 也是最要命的一处 ★
 *      原版状态0 要求 Search_Stop_Line >= H*0.95 (= 120 行的 114)。
 *      照搬过来就是 IMG_ROW*0.95 = 57。
 *      但本车实测: 直道 SS 最大只有 55, 环岛入口 SS=53 ——
 *      门槛 57 永远达不到, 于是状态0 一帧都通不过, 永远进不了状态1。
 *      这里改用工程里已经标定好的 ISL_STOP_MIN (= IMG_ROW*0.80 = 48),
 *      和 Island_Detect() 用的是同一个宏, 改一处两个版本一起变。
 *
 *  ★ 第六处 —— 状态1 卡死 ★
 *      原版 状态1->2 是 Boundry_Start_Left < 30 (120 行图)。
 *      按分辨率减半成 15 是错的! 本工程实测(island.c 顶部 ISL_DOWN_BS 注释):
 *          车停在环岛口 BS=59, 往前开、下角点消失的一瞬间 BS 掉到 28。
 *      阈值 15 永远等不到 -> 卡死在状态1 -> 车用断掉的边界算中线 -> 不走直线。
 *      改用工程实测好的 ISL_DOWN_BS(=30), 左右环岛都是。
 *
 *  ★ 第七处 —— 状态1/2 的补线画不出来 ★
 *      原版直接调 Monotonicity_Change_*(15,2)。本工程的 Monotonicity_Change_*
 *      内部会用 isl_clamp_range 把范围夹进"边界数组真正被赋过值"的区域,
 *      夹的时候多留了 ISL_TOP_MARGIN(=8) 行:
 *          top = IMG_ROW - SS + 2 + 8;  SS=53 -> 17
 *          17 > start(15) -> top=15;  end(5) < 15 -> end=15
 *          start(15) <= end(15) -> 直接 return 0
 *      要过这条得 SS>55, 而本车 SS 最大就是 55 —— 恒为 0, 补线一帧都画不出。
 *      所以新增 Monotonicity_Change_*_Ex(start, end, top_skip), 状态1 传 top_skip=0:
 *          SS=53 -> top=9, 扫 15~9 行, i±ISL_WIN 最远用到第 7 行, 仍在有效区内。
 *      状态2 的搜索范围是 (35,5), start 够大, 不受影响, 保持原样。
 *
 *  ★ 这个版本没有"入环确认帧数"防抖 ★
 *      斜入十字时可能会误判进状态1。要防抖的话告诉我, 照 Isl_Confirm_Frames 加一份。
 *
 *  用法: main.c 里把 Island_Detect(); 换成 Island_Detect3();
 * ========================================================================================== */

/* 注: 原来这里有个 isl3_edge_white() —— 给"角点连到图像左下角白点"那条补线
       找下端锚点用的。状态3 改成统一顺延边界之后它没人调用了, 已删。
       那套"角点连线"的代码也没了, 但 Find_*_Up_Point_Ex 和 ISL3_S3_UP_*
       还留着 —— 想换回去随时可以用。 */

void Island_Detect3(void)
{
    static float k = 0.0f;                      /* 状态5/6 用的斜率(出环补线) */
    static int   island_state_5_down[2] = {0};  /* 状态5: 即将出环时边界的最低点 [0]=行 [1]=列 */
    static int   left_down_guai[2]      = {0};  /* 认定环岛时用的左下角点 */
    static int   right_down_guai[2]     = {0};  /* 认定环岛时用的右下角点 */

    /* ★ 斜十字防误判: 入环确认计数 ★

       判据必须【连续】Isl_Confirm_Frames 帧都成立, 才真的认定是环岛。

       斜着进十字时, 车会先看到十字的一个角 —— 那一两帧的图像特征
       (一侧不连续 + 下角点靠下) 和环岛入口几乎一模一样。不加防抖就会误进状态1,
       车速被压到 Island_Speed, 环岛状态机整个跑偏。
       而真环岛的这个特征是【持续存在】的, 十字的角只是路过 —— 这就是判据。

       Isl_Confirm_Frames 就是菜单3 的 "ISL CFM"(默认 6), 和 Island_Detect 共用,
       改一处两个版本一起变。 */
    static uint8 isl3_left_hit  = 0;
    static uint8 isl3_right_hit = 0;

    int monotonicity_change_line[2];            /* 单调性突变点 [0]=行 [1]=列 */
    int monotonicity_change_left_flag  = 0;
    int monotonicity_change_right_flag = 0;
    int continuity_change_right_flag   = 0;
    int continuity_change_left_flag    = 0;

    /* ---------------- 四个基础判据 ---------------- */
    continuity_change_left_flag    = Continuity_Change_Left (IMG_ROW - 1 - 2, 5);   /* 原 (H-1-5, 10) */
    continuity_change_right_flag   = Continuity_Change_Right(IMG_ROW - 1 - 2, 5);
    monotonicity_change_right_flag = Monotonicity_Change_Right(IMG_ROW - 1 - 5, 5); /* 原 (H-1-10, 10) */
    monotonicity_change_left_flag  = Monotonicity_Change_Left (IMG_ROW - 1 - 5, 5);

    Island_Dbg_CL = continuity_change_left_flag;
    Island_Dbg_CR = continuity_change_right_flag;
    Island_Dbg_ML = monotonicity_change_left_flag;
    Island_Dbg_MR = monotonicity_change_right_flag;

    /* 状态9 是"刚跑完", 这里归零防止连续判环 */
    if (Island_State == 9)
    {
        Island_State = 0;
        Island_Over_Flag = 1;
    }

    /* ==================================================================================
     *  状态0: 认定环岛
     * ================================================================================== */
    /* 诊断: 每次"重新进入状态1"就把持续帧数清零重数。
       只有累加的话分不清"卡在状态1"和"跑了好几圈正常累加", 清零之后
       DUR 就是最近这一次状态1 的长度: 几十 = 正常穿过, 99 = 卡住出不来。 */
    {
        static int isl3_prev_state = -1;

        if ((Island_State == 1) && (isl3_prev_state != 1))
        {
            Isl_Diag_S1_DUR = 0;
        }
        isl3_prev_state = Island_State;
    }

    if (Cross_Flag == 0 && Island_State == 0 && Ramp_Flag == 0)
    {
        /* ---------------- 左环岛 ---------------- */
        if (Left_Island_Flag == 0)
        {
            if (monotonicity_change_right_flag == 0      &&   /* 右边是单调的 */
                continuity_change_left_flag    != 0      &&   /* 左边是不连续的 */
                continuity_change_right_flag   == 0      &&   /* 左环岛右边是连续的 */
                Left_Lost_Time  >= 5                     &&   /* 原 10, 左边丢线很多 */
                Left_Lost_Time  <= 25                    &&   /* 原 50, 也不能全丢了 */
                Right_Lost_Time <= 5                     &&   /* 原 10, 右边丢线较少 */
                Search_Stop_Line >= ISL_STOP_MIN         &&   /* ★ 见下 ★ */
                Boundry_Start_Left  >= IMG_ROW - 10      &&   /* 原 H-20, 边界起始点靠下 */
                Boundry_Start_Right >= IMG_ROW - 10      &&
                Both_Lost_Time <= 5)                          /* 原 10, 双边丢线少 */
            {
                left_down_guai[0] = Find_Left_Down_Point(IMG_ROW - 1, 10);  /* 原 (H-1, 20) */

                if (left_down_guai[0] >= 15)                /* 原 30, 拐点位置不对就是误判 */
                {
                    /* ★ 连续 Isl_Confirm_Frames 帧都成立才真的认定 ★ */
                    if (isl3_left_hit < Isl_Confirm_Frames)
                    {
                        isl3_left_hit ++;
                    }

                    if (isl3_left_hit >= Isl_Confirm_Frames)
                    {
                        Island_State     = 1;
                        Left_Island_Flag = 1;
                        FJ_Angle         = 0.0f;    /* ★ 本工程没陀螺仪, 入环必须清零 ★ */
                        isl3_left_hit    = 0;
                    }
                }
                else
                {
                    isl3_left_hit    = 0;           /* 拐点位置不对, 重新数 */
                    Island_State     = 0;
                    Left_Island_Flag = 0;
                }
            }
            else
            {
                isl3_left_hit = 0;                  /* 判据断了就重新数 */
            }
        }

        /* ---------------- 右环岛 ---------------- */
        if (Right_Island_Flag == 0)
        {
            if (monotonicity_change_left_flag == 0     &&
                continuity_change_left_flag   == 0     &&   /* 右环岛左边是连续的 */
                continuity_change_right_flag  != 1     &&   /* 右边是不连续的 */
                Right_Lost_Time >= 5                   &&   /* 原 10 */
                Right_Lost_Time <= 25                  &&   /* 原 50 */
                Left_Lost_Time  <= 5                   &&   /* 原 10 */
                Search_Stop_Line >= ISL_STOP_MIN       &&   /* ★ 见下 ★ */
                Boundry_Start_Left  >= IMG_ROW - 10    &&
                Boundry_Start_Right >= IMG_ROW - 10    &&
                Both_Lost_Time <= 5)
            {
                right_down_guai[0] = Find_Right_Down_Point(IMG_ROW - 1, 10);

                if (right_down_guai[0] >= 15)
                {
                    /* ★ 同左环岛: 连续 Isl_Confirm_Frames 帧才认定 ★ */
                    if (isl3_right_hit < Isl_Confirm_Frames)
                    {
                        isl3_right_hit ++;
                    }

                    if (isl3_right_hit >= Isl_Confirm_Frames)
                    {
                        Island_State      = 1;
                        Right_Island_Flag = 1;
                        FJ_Angle          = 0.0f;   /* ★ 入环清零 ★ */
                        isl3_right_hit    = 0;
                    }
                }
                else
                {
                    isl3_right_hit    = 0;          /* 拐点位置不对, 重新数 */
                    Island_State      = 0;
                    Right_Island_Flag = 0;
                }
            }
            else
            {
                isl3_right_hit = 0;                 /* 判据断了就重新数 */
            }
        }
    }
    else
    {
        /* 不满足状态0 的前提(正在十字里 / 已经在环岛里 / 坡道) —— 确认计数一律清零。

           ★ 这一条才是防斜入十字误判的关键 ★
           斜着进十字时, Cross_Flag 可能在 0/1 之间来回跳。
           如果只在"正在十字里"那几帧清零, 计数就会在那些"恰好 Cross_Flag==0"
           的帧上偷偷累加 —— Isl_Confirm_Frames 设多大都拦不住。 */
        isl3_left_hit  = 0;
        isl3_right_hit = 0;
    }

    /* ==================================================================================
     *  左环岛状态机
     * ================================================================================== */
    if (Left_Island_Flag == 1)
    {
        /* ---- 状态1: 拐点还在, 没丢线 ---- */
        if (Island_State == 1)
        {
            /* ★ 必须用 _Ex(...,0) 不能用普通版 ★
               普通版内部 top_skip = ISL_TOP_MARGIN(8), 会把扫描范围夹成
               start == end 然后直接返回 0 —— 补线一帧都画不出来。
               原因见 Monotonicity_Change_Left_Ex 上面的注释。 */
            monotonicity_change_line[0] = Monotonicity_Change_Left_Ex(ISL3_S1_SEARCH, 2, 0, ISL3_PATCH_PROM);

            /* 菜单1 的 ML 只在"这一帧正好是状态1"时才是这个搜索结果,
               平时都被状态0 的值盖掉了 —— 所以真正要看的是菜单4 的锁存值。
               这里顺手也写一份, 便于单帧对照。 */
            Island_Dbg_ML = monotonicity_change_line[0];

            /* ---- 锁存诊断快照 (菜单4 显示): 只记"刚进入状态1 的第一帧" ----
               ★ 为什么限定第一帧 ★
               状态1 实测能持续 99 帧。如果每帧都覆盖, 最后留下的是"马上要转状态2、
               本侧边界已经丢了一大片"那一帧 —— 那时搜索窗(第 7~15 行)整段都落在
               丢线区里, 记下来的失败原因不能代表补线真正该工作的时刻。
               第一帧才是刚认定环岛、边界还完好的时候, 也正是补线该起作用的时刻。 */
            if (Isl_Diag_S1_DUR == 0)
            {
                int b;

                Isl_Diag_S1_LR  = 1;                    /* 1 = 左环岛 */
                Isl_Diag_S1_L   = monotonicity_change_line[0];
                Isl_Diag_S1_SS  = Search_Stop_Line;
                Isl_Diag_S1_BS  = Boundry_Start_Left;   /* 本侧, 状态1->2 就看它 <= 30 */
                Isl_Diag_S1_LT  = Left_Lost_Time;       /* 本侧丢线帧数 */
                Isl_Diag_S1_WHY = Isl_Mono_Why;         /* 搜索失败原因码 */

                /* 和 isl_clamp_range 在 top_skip=0 时的算法完全一致:
                   top = IMG_ROW - SS + idx_margin(ISL_WIN), 再夹到 [ISL_TOP_ROWS, start] */
                {
                    int t = IMG_ROW - Search_Stop_Line + ISL_WIN;
                    if (t < ISL_TOP_ROWS)     { t = ISL_TOP_ROWS; }
                    if (t > ISL3_S1_SEARCH)   { t = ISL3_S1_SEARCH; }
                    Isl_Diag_S1_TOP = t;
                }

                /* 把搜索窗那 9 行的边界原值原样抄下来 —— 搜不到的原因看这 9 个数最直接 */
                for (b = 0; b < ISL3_BAND_N; b ++)
                {
                    Isl_Diag_S1_BAND[b] = Left_Line[ISL3_BAND_ROW0 + b];
                }
            }

            if ((monotonicity_change_line[0] > 0) && (Isl_Diag_S1_HIT < 99))
            {
                Isl_Diag_S1_HIT ++;                     /* 整个状态1 期间搜到过几帧 */
            }
            if (Isl_Diag_S1_DUR < 99) { Isl_Diag_S1_DUR ++; }

            /* ★ 找不到转折点(mcl[0]==0)绝对不要补线 ★ 原版没有这个判断, 必须补上:
               此时 Left_Line[0] 不是真的边界, 而 y2=0 会让 Left_Add_Line
               把整条左边界从第 0 行到第 59 行改写成一条直线。
               边界一毁, 角点判据必然返回 0, 状态机会一路冲到状态 3。
               (工程里原有的 Island_Detect 在同样位置就是这么防的) */
            if (monotonicity_change_line[0] > 0)
            {
                /* 原版做法: 单调突变点向下连线 */
                monotonicity_change_line[1] = Left_Line[monotonicity_change_line[0]];
                Left_Add_Line((int)(monotonicity_change_line[1] * 0.15f), IMG_ROW - 1,
                              monotonicity_change_line[1], monotonicity_change_line[0]);
            }
            else if ((Boundry_Start_Left > 1) && (Boundry_Start_Left < IMG_ROW))
            {
                /* ★ 兜底: 顺延边界 (原版没有, 但十字就是这么干的) ★

                   十字检测在"凑不齐四个角点"时会用 Lengthen_Left/Right_Boundry
                   把边界顺着已有斜率延长过去, 所以它永远有东西可画。
                   状态1 原来是"搜不到单调突变点就什么都不做" —— 一帧补线都没有,
                   车拿着断掉的边界算中线, 当然进不去环岛。

                   Boundry_Start_Left 是"本侧边界最低的那一行有效行",
                   它下面(BS+1 ~ 59)整段都丢线。从 BS-1 顺着已有斜率延长到底,
                   正好把断掉的那一段接上。 */
                Lengthen_Left_Boundry(Boundry_Start_Left - 1, IMG_ROW - 1);
            }

            /* ★ 不能用减半后的 15 ★
               这一条本工程早就实测过(见 island.c 顶部 ISL_DOWN_BS 的注释):
               车停在环岛口时 BS=59, 往前开、下角点消失的一瞬间 BS 掉到 28。
               阈值取 15 的话永远等不到 -> 卡死在状态1, 补线又没有,
               中线用断掉的边界算出来 -> 车不走直线。
               所以直接用工程里实测好的 ISL_DOWN_BS(=30)。 */
            if (Boundry_Start_Left <= ISL_DOWN_BS)      /* 原 < 30 */
            {
                Island_State = 2;
            }
        }

        /* ---- 状态2: 下方角点消失, 上面是弧线 ---- */
        else if (Island_State == 2)
        {
            /* ★ 只在"本侧边界真正有效"的行里搜 ★

               Boundry_Start_Left 的定义(见 Longest_White_Column):
               从第 59 行往上扫, 第一行"没丢线"的行号 —— 也就是本侧边界
               最低的那一行有效行。它下面(BS+1 ~ 59)整段都是丢线。
               而丢线行的 Left_Line 是重建边界时留下的默认值 0
               (右边界是 IMG_COL-1), 拿它当锚点会补出一条贴在图像边上的废线。

               所以把搜索起点压到 BS 以内。压完如果起点跑到终点上面去了,
               _Ex 会返回 0, 自动走下面的兜底。

               另外和状态1 一样改用 _Ex: 普通版写死了 top_skip=8 和
               ISL_MONO_PROM=3, 会把范围夹窄、并把真实拐点当毛刺滤掉。
               top_skip 传 0 更接近原版(原版范围 70~10)。 */
            {
                int s = ISL3_S2_SEARCH;
                if (s > Boundry_Start_Left - 1) { s = Boundry_Start_Left - 1; }
                monotonicity_change_line[0] = Monotonicity_Change_Left_Ex(s, 5, 0, ISL3_PATCH_PROM);
            }

            if (monotonicity_change_line[0] > 0)
            {
                /* 原版做法: 单调突变点向下连线 */
                monotonicity_change_line[1] = Left_Line[monotonicity_change_line[0]];
                Left_Add_Line((int)(monotonicity_change_line[1] * 0.1f), IMG_ROW - 1,
                              monotonicity_change_line[1], monotonicity_change_line[0]);
            }
            else if ((Boundry_Start_Left > 1) && (Boundry_Start_Left < IMG_ROW))
            {
                /* ★ 兜底: 顺延边界 —— 和状态1、和十字的兜底是同一套思路 ★
                   状态2 下角点已经丢了、上方开始出现弧线。
                   Boundry_Start_Left 是"最低的那一行有效行", 从它上面一行
                   顺着已有斜率延长到底即可。 */
                Lengthen_Left_Boundry(Boundry_Start_Left - 1, IMG_ROW - 1);
            }

            /* ★ 出口: 单调突变点的行号大于 ISL3_S2_EXIT_ROW 就进状态3 ★
               行号越大 = 越靠近车(画面下方), 说明要进环岛了。
               原来的 (mcl[0] > 25) 已按你的要求换成这个。
               后面那条 Boundry_Start 是原版的条件, 留着当保险 ——
               它要求 BS>=58, 而 mcl[0]>45 要求 BS>46, 所以新判据总是先成立,
               正常情况下不会由它触发。 */
            if ((monotonicity_change_line[0] > ISL3_S2_EXIT_ROW) ||
                (Boundry_Start_Left >= IMG_ROW - 2))            /* 原 H-5, 保险 */
            {
                Island_State     = 3;
                Left_Island_Flag = 1;
            }
        }

        /* ---- 状态3: 锁定最长白列 + 顺延边界 ---- */
        else if (Island_State == 3)
        {
            /* ★ 状态3 的补线统一改成【延长边界】★
               (不再用"角点连到图像左下/右下角白点"那种画法)

               做法和状态1/2、和十字的兜底完全一样:
               从本侧边界最低的那一行有效行, 顺着已有斜率延长到底。

               为什么废掉角点连线:
                 ① 锁定最长白列之后, 角点落在很靠上、边界很平缓的行,
                    形状判据本来就很难命中 —— 实测一直在走兜底;
                 ② 就算命中, 那条线是从图像边角拉出来的, 和边界本身的
                    走向不连续, 进中线之后容易让舵机抖。
               顺延边界至少保证和已有边界是连续的。 */

            /* ① 重扫边界 —— 让锁定立刻生效, 不用等下一帧 */
            Longest_White_Column();

            /* ② 顺延边界

                  Boundry_Start_Right 是"本侧边界最低的那一行有效行",
                  它下面(BS+1 ~ 59)整段丢线, 所以 BS < IMG_ROW-1 才需要补;
                  要是根本没丢线, 就什么都不用做。 */
            if ((Boundry_Start_Right > 1) && (Boundry_Start_Right < IMG_ROW - 1))
            {
                Lengthen_Right_Boundry(Boundry_Start_Right - 1, IMG_ROW - 1);
            }

            /* ④ 角度积分到 60 度就进状态4 */
            if (abs((int)FJ_Angle) >= 60)       /* 原 abs(FJ_Angle)>=60, 靠角度积分入环 */
            {
                k = 0.0f;
                Island_State = 4;
            }
        }

        /* ---- 状态4: 已经在里面 ---- */
        else if (Island_State == 4)
        {
            if (abs((int)FJ_Angle) > 200)       /* 积分 200 度后再打开出环判断 */
            {
                /* 和状态1/2 一样改用 _Ex: 普通版写死了 prom=ISL_MONO_PROM(3),
                   会把真实拐点当毛刺滤掉; top_skip 传 0 也更接近原版的范围 */
                monotonicity_change_line[0] =
                    Monotonicity_Change_Right_Ex(IMG_ROW - 5, 5, 0, ISL3_PATCH_PROM); /* 原 (H-10,10) */
                monotonicity_change_line[1] = Right_Line[monotonicity_change_line[0]];

                if ((monotonicity_change_line[0] >= 17) &&
                    (monotonicity_change_line[0] <= 27) &&      /* 原 35~55 */
                    (monotonicity_change_line[1] >= 5))         /* 原 >= 10 */
                {
                    island_state_5_down[0] = IMG_ROW - 1;
                    island_state_5_down[1] = Right_Line[IMG_ROW - 1];

                    if (island_state_5_down[1] != monotonicity_change_line[1])  /* 除零保护 */
                    {
                        k = (float)(IMG_ROW - monotonicity_change_line[0])
                          / (float)(island_state_5_down[1] - monotonicity_change_line[1]);

                        K_Add_Boundry_Right(k, island_state_5_down[1], island_state_5_down[0], 0);
                        Island_State = 5;
                    }
                }
            }
        }

        /* ---- 状态5: 出环 ---- */
        else if (Island_State == 5)
        {
            K_Add_Boundry_Right(k, island_state_5_down[1], island_state_5_down[0], 0);

            if (Boundry_Start_Right < IMG_ROW - 10)     /* 原 H-20, 右边先丢线 */
            {
                Island_State = 6;
            }
        }

        /* ---- 状态6: 还在出 ---- */
        else if (Island_State == 6)
        {
            K_Add_Boundry_Right(k, island_state_5_down[1], island_state_5_down[0], 0);

            if ((Boundry_Start_Right > IMG_ROW - 5) ||  /* 原 H-10, 右边不丢线了 */
                (abs((int)FJ_Angle) >= 320))
            {
                k = 0.0f;
                Island_State = 7;
            }
        }

        /* ---- 状态7: 基本出去了, 找拐点 ---- */
        else if (Island_State == 7)
        {
            Left_Up_Guai[0] = Find_Left_Up_Point(IMG_ROW - 5, 0);   /* 原 (H-10, 0) */
            Left_Up_Guai[1] = Left_Line[Left_Up_Guai[0]];

            if ((Left_Up_Guai[1] <= 50) &&                          /* 原 <= 100 */
                (Left_Up_Guai[0] >= 2) && (Left_Up_Guai[0] <= IMG_ROW - 10))  /* 原 5 ~ H-20 */
            {
                Island_State = 8;
            }
        }

        /* ---- 状态8: 连线, 出环最后一步 ---- */
        else if (Island_State == 8)
        {
            Left_Up_Guai[0] = Find_Left_Up_Point(IMG_ROW - 1, 5);   /* 原 (H-1, 10) */
            Left_Up_Guai[1] = Left_Line[Left_Up_Guai[0]];

            /* ★ 找不到拐点(返回 0)时绝对不能补 ★
               Guai[0]-1 = -1 会被钳成 0, 而 Lengthen 在 start<=5 时走的是
               "直线延长"分支 —— 它会拿 Left_Line[IMG_ROW-1] 当终点,
               拉出一条和真实赛道毫无关系的线。 */
            if (Left_Up_Guai[0] > 1)
            {
                Lengthen_Left_Boundry(Left_Up_Guai[0] - 1, IMG_ROW - 1);
            }

            if ((Left_Up_Guai[0] >= IMG_ROW - 10) ||                            /* 原 H-20 */
                ((Left_Up_Guai[0] < 5) && (Boundry_Start_Left >= IMG_ROW - 5))) /* 原 <10, H-10 */
            {
                FJ_Angle          = 0.0f;
                Island_State      = 9;
                Left_Island_Flag  = 0;
            }
        }
    }
    /* ==================================================================================
     *  右环岛状态机
     * ================================================================================== */
    else if (Right_Island_Flag == 1)
    {
        /* ---- 状态1: 拐点还在, 没丢线 ---- */
        if (Island_State == 1)
        {
            monotonicity_change_line[0] = Monotonicity_Change_Right_Ex(ISL3_S1_SEARCH, 2, 0, ISL3_PATCH_PROM);

            /* 同左环岛: 菜单1 的 MR 每帧都会被状态0 的值盖掉, 真正要看的是菜单4 */
            Island_Dbg_MR = monotonicity_change_line[0];

            /* ---- 锁存诊断快照: 只记第一帧, 原因见左环岛 ---- */
            if (Isl_Diag_S1_DUR == 0)
            {
                int b;

                Isl_Diag_S1_LR  = 2;                    /* 2 = 右环岛 */
                Isl_Diag_S1_R   = monotonicity_change_line[0];
                Isl_Diag_S1_SS  = Search_Stop_Line;
                Isl_Diag_S1_BS  = Boundry_Start_Right;  /* 本侧, 状态1->2 就看它 <= 30 */
                Isl_Diag_S1_LT  = Right_Lost_Time;      /* 本侧丢线帧数 */
                Isl_Diag_S1_WHY = Isl_Mono_Why;         /* 搜索失败原因码 */

                {
                    int t = IMG_ROW - Search_Stop_Line + ISL_WIN;
                    if (t < ISL_TOP_ROWS)     { t = ISL_TOP_ROWS; }
                    if (t > ISL3_S1_SEARCH)   { t = ISL3_S1_SEARCH; }
                    Isl_Diag_S1_TOP = t;
                }

                for (b = 0; b < ISL3_BAND_N; b ++)
                {
                    Isl_Diag_S1_BAND[b] = Right_Line[ISL3_BAND_ROW0 + b];
                }
            }

            if ((monotonicity_change_line[0] > 0) && (Isl_Diag_S1_HIT < 99))
            {
                Isl_Diag_S1_HIT ++;
            }
            if (Isl_Diag_S1_DUR < 99) { Isl_Diag_S1_DUR ++; }

            if (monotonicity_change_line[0] > 0)
            {
                /* 原版做法: 单调突变点向下连线 */
                monotonicity_change_line[1] = Right_Line[monotonicity_change_line[0]];
                Right_Add_Line((int)(IMG_COL - 1 - (monotonicity_change_line[1] * 0.15f)), IMG_ROW - 1,
                               monotonicity_change_line[1], monotonicity_change_line[0]);
            }
            else if ((Boundry_Start_Right > 1) && (Boundry_Start_Right < IMG_ROW))
            {
                /* ★ 兜底: 顺延边界, 和左环岛、和十字的兜底是同一套思路 ★ */
                Lengthen_Right_Boundry(Boundry_Start_Right - 1, IMG_ROW - 1);
            }

            if (Boundry_Start_Right <= ISL_DOWN_BS)     /* 原 30, 原因见左环岛 */
            {
                Island_State = 2;
            }
        }

        /* ---- 状态2: 下方丢线, 上方即将出现大弧线 ---- */
        else if (Island_State == 2)
        {
            /* 同左环岛状态2: 起点压到有效区内 + 改用 _Ex, 原因见那边 */
            {
                int s = ISL3_S2_SEARCH;
                if (s > Boundry_Start_Right - 1) { s = Boundry_Start_Right - 1; }
                monotonicity_change_line[0] = Monotonicity_Change_Right_Ex(s, 2, 0, ISL3_PATCH_PROM);
            }


            if (monotonicity_change_line[0] > 0)
            {
                /* 原版做法: 单调突变点向下连线 */
                monotonicity_change_line[1] = Right_Line[monotonicity_change_line[0]];
                Right_Add_Line((int)(IMG_COL - 1 - (monotonicity_change_line[1] * 0.15f)), IMG_ROW - 1,
                               monotonicity_change_line[1], monotonicity_change_line[0]);
            }
            else if ((Boundry_Start_Right > 1) && (Boundry_Start_Right < IMG_ROW))
            {
                /* ★ 兜底: 顺延边界, 同左环岛 ★ */
                Lengthen_Right_Boundry(Boundry_Start_Right - 1, IMG_ROW - 1);
            }

            /* ★ 出口: 同左环岛 —— 突变点行号 > ISL3_S2_EXIT_ROW 进状态3 ★ */
            if ((monotonicity_change_line[0] > ISL3_S2_EXIT_ROW) ||
                (Boundry_Start_Right >= IMG_ROW - 2))           /* 原 H-5, 保险 */
            {
                Island_State      = 3;
                Right_Island_Flag = 1;
            }
        }

        /* ---- 状态3: 锁定最长白列 + 顺延边界 ---- */
        else if (Island_State == 3)
        {
            /* 同左环岛状态3, 原因见那边 */

            /* ① 重扫边界, 让锁定立刻生效 */
            Longest_White_Column();

            /* ② 顺延边界

                  Boundry_Start_Left 是"本侧边界最低的那一行有效行",
                  它下面(BS+1 ~ 59)整段丢线, 所以 BS < IMG_ROW-1 才需要补;
                  要是根本没丢线, 就什么都不用做。 */
            if ((Boundry_Start_Left > 1) && (Boundry_Start_Left < IMG_ROW - 1))
            {
                Lengthen_Left_Boundry(Boundry_Start_Left - 1, IMG_ROW - 1);
            }

            /* ④ 角度积分到 60 度进状态4 */
            if (abs((int)FJ_Angle) >= 60)       /* 只依靠角度积分 */
            {
                k = 0.0f;
                Island_State = 4;
            }
        }

        /* ---- 状态4: 完全进去环岛了 ---- */
        else if (Island_State == 4)
        {
            if (abs((int)FJ_Angle) > 200)       /* 积分 200 度后再打开出环判断 */
            {
                /* 和状态1/2 一样改用 _Ex, 原因见左环岛状态4 */
                monotonicity_change_line[0] =
                    Monotonicity_Change_Left_Ex(45, 5, 0, ISL3_PATCH_PROM);  /* 原 (90,10) */
                monotonicity_change_line[1] = Left_Line[monotonicity_change_line[0]];

                if ((monotonicity_change_line[0] >= 17) &&
                    (monotonicity_change_line[0] <= 27) &&          /* 原 35~55 */
                    (monotonicity_change_line[1] < IMG_COL - 5))    /* 原 < W-10 */
                {
                    island_state_5_down[0] = IMG_ROW - 1;
                    island_state_5_down[1] = Left_Line[IMG_ROW - 1] - 3;    /* 原 -5 */

                    if (island_state_5_down[1] != monotonicity_change_line[1])
                    {
                        k = (float)(IMG_ROW - monotonicity_change_line[0])
                          / (float)(island_state_5_down[1] - monotonicity_change_line[1]);

                        K_Add_Boundry_Left(k, island_state_5_down[1], island_state_5_down[0], 0);
                        Island_State = 5;
                    }
                }
            }
        }

        /* ---- 状态5: 准备出环岛 ---- */
        else if (Island_State == 5)
        {
            K_Add_Boundry_Left(k, island_state_5_down[1], island_state_5_down[0], 0);

            if (Boundry_Start_Left < IMG_ROW - 10)      /* 原 H-20, 左边先丢线 */
            {
                Island_State = 6;
            }
        }

        /* ---- 状态6: 继续出 ---- */
        else if (Island_State == 6)
        {
            K_Add_Boundry_Left(k, island_state_5_down[1], island_state_5_down[0], 0);

            if ((Boundry_Start_Left > IMG_ROW - 5) ||   /* 原 H-10 */
                (abs((int)FJ_Angle) >= 320))
            {
                k = 0.0f;
                Island_State = 7;
            }
        }

        /* ---- 状态7: 基本出环岛, 找角点 ---- */
        else if (Island_State == 7)
        {
            Right_Up_Guai[0] = Find_Right_Up_Point(IMG_ROW - 5, 0);     /* 原 (H-10, 0) */
            Right_Up_Guai[1] = Right_Line[Right_Up_Guai[0]];

            if ((Right_Up_Guai[1] >= IMG_COL - 44) &&                   /* 原 >= W-88 */
                (Right_Up_Guai[0] >= 2) && (Right_Up_Guai[0] <= IMG_ROW - 10))  /* 原 5 ~ H-20 */
            {
                Island_State = 8;
            }
        }

        /* ---- 状态8: 出环最后一步 ---- */
        else if (Island_State == 8)
        {
            Right_Up_Guai[0] = Find_Right_Up_Point(IMG_ROW - 1, 5);     /* 原 (H-1, 10) */
            Right_Up_Guai[1] = Right_Line[Right_Up_Guai[0]];

            /* ★ 找不到拐点时不能补, 原因见左环岛状态8 ★ */
            if (Right_Up_Guai[0] > 1)
            {
                Lengthen_Right_Boundry(Right_Up_Guai[0] - 1, IMG_ROW - 1);
            }

            if ((Right_Up_Guai[0] >= IMG_ROW - 10) ||                           /* 原 H-20 */
                ((Right_Up_Guai[0] < 5) && (Boundry_Start_Left >= IMG_ROW - 5))) /* 原 <10, H-10 */
            {
                FJ_Angle          = 0.0f;
                Island_State      = 9;
                Right_Island_Flag = 0;
            }
        }
    }
}
