#include "tft.h"
#include "zf_common_headfile.h"
#include "Ourcode_headfile.h"

/* ==========================================================================================
 *  屏幕菜单  (TFT180  160 x 128  横屏, 6x8 字体)
 *
 *  6x8 字体下一行 26 个字符, 行高 8 像素, 一共 16 行(0 ~ 15)
 *
 *  按键分配
 *      KEY6            循环切换菜单   1 -> 2 -> 3 -> 4 -> 5 -> 1
 *      KEY7            启动 / 停止(翻转 GO), 任何页面都有效
 *      菜单 2 / 3      KEY1 上移   KEY4 下移   KEY2 换步长   KEY3 加   KEY5 减
 *      菜单 4          KEY1 上移   KEY4 下移   KEY2 执行选中项
 *      菜单 5          KEY2 开始采集   KEY5 清空结果
 *
 *  菜单 1  摄像头原图 + 二值图 + 边线 + 实时数据
 *  菜单 2  舵机 / 左右电机 PID 在线调参
 *  菜单 3  运行参数在线调参
 *  菜单 4  电机测试 + 恢复默认值 + 按键电平监视
 *  菜单 5  直道标准赛宽标定(采集 32 帧, 屏幕上直接显示成 C 数组供手抄)
 * ========================================================================================== */

/* ---------------------------------------------------------------- 屏幕参数 */
#define LINE_H              8                   /* 6x8 字体行高 */
#define LINE_CHAR           26                  /* 160 / 6 = 26 个字符 */

#define TEXT_ROW(row)       ((uint16)((row) * LINE_H))

/* 数值显示区: 从第 17 列开始 */
#define VAL_COL             17
#define VAL_X               (VAL_COL * 6)       /* 102 */

#define VAL_INT_DIGIT       4                   /* 整数位, 也用作 tft180_show_int 的位数 */
#define VAL_WIDTH           9                   /* 数值区宽度(字符) = 4 整数位 + '.' + 3 小数位 */

/* ---------------------------------------------------------------- 配色 */
#define COL_TITLE_PEN       RGB565_BLUE
#define COL_TITLE_BG        RGB565_WHITE
#define COL_TEXT_PEN        RGB565_BLACK
#define COL_TEXT_BG         RGB565_WHITE
#define COL_SEL_PEN         RGB565_WHITE
#define COL_SEL_BG          RGB565_BLUE

/* ---------------------------------------------------------------- 菜单页数 */
#define MENU_PAGE_NUM       6

/* ---------------------------------------------------------------- 外部变量 */
extern volatile float   angle;
extern volatile int     GO;
extern volatile float   Err;
extern volatile int     Speed_Left_Real;
extern volatile int     Speed_Right_Real;
extern volatile int     Cross_Flag;             /* 定义在 img_process.c */

/* ---------------------------------------------------------------- 菜单状态 */
static uint8    menu_page  = 1;
static uint8    page_dirty = 1;                 /* 1 = 本帧需要整页重画 */

/* ---------------------------------------------------------------- 帧率统计
 *
 *  tft_show() 在 main 的循环里每个摄像头帧被调用一次, 所以数它的调用次数就是帧率。
 *  时间基准借用 TIM5 的 1ms 计数 sys_ms(wheel.h 里声明的那个),
 *  每 FPS_WIN_MS 毫秒结算一次, 这样读数不会每帧乱跳。
 *
 *  为什么要看这个: 调 -O2 / 改刷屏逻辑 / 换图像算法之后, 屏幕上能直接看出快了多少。
 */
#define FPS_WIN_MS      500                     /* 统计窗口, 毫秒。调小反应快但跳, 调大稳但迟钝 */

/* 帧率显示在两个地方:
     菜单1 第 11 行 —— 那一页要画整幅灰度图, 是最费时间的页面, 读数是"最坏情况"
     菜单3 第  8 行 —— 那一页只在换页时重画, 平时每帧只刷这 3 个字符,
                       几乎不占时间, 所以读数是"真实帧率", 跑车时看这个
     两个数一对比, 就知道刷屏到底吃掉了多少帧率 */
#define FPS_COL         5                       /* 数值从第 5 列开始 */
#define FPS_X           (FPS_COL * 6)           /* 30 px */

static uint32   fps_cnt     = 0;                /* 本窗口内累计的帧数 */
static uint32   fps_t0      = 0;                /* 本窗口的起始时刻 */
static uint32   fps_val     = 0;                /* 最近一次结算出来的帧率 */
static uint8    fps_started = 0;                /* 已经丢掉过第一个不完整的窗口 */

static void fps_tick(void)
{
    uint32 now = sys_ms;
    uint32 span;

    fps_cnt ++;

    /* sys_ms 是 uint32, 这里必须用"无符号减法"来算间隔。
       直接写 (now >= fps_t0 + FPS_WIN_MS) 的话, sys_ms 跑满 49.7 天回绕时会卡住;
       无符号减法在回绕时结果依然正确。 */
    span = now - fps_t0;
    if (span >= FPS_WIN_MS)
    {
        /* 只有"正常长度"的窗口才算数:
             - 第一个窗口是从上电(sys_ms=0)开始算的, 但摄像头初始化完之后
               tft_show() 才开始被调用, 所以这一段跨度对不上, 结果是偏低的假数据;
             - 中间要是被卡住超过 2 秒(长按、清屏、或者别的地方阻塞了), 那一段同样不算。
           这两种情况都只把窗口重新对齐, 不更新读数, 免得屏幕上闪一个误导人的低值。 */
        if (fps_started && (span < FPS_WIN_MS * 4u))
        {
            fps_val = (fps_cnt * 1000u) / span;
        }

        fps_started = 1;
        fps_cnt     = 0;
        fps_t0      = now;
    }
}

/* 调参步长 */
static const float      step_tab[3] = {0.1f, 1.0f, 10.0f};
static const char *const step_str[3] = {"0.1", "1.0", "10"};
static uint8    step_idx = 1;

/* 恢复默认值用 */
#define DEF_BASE_SPD        90.0f
#define DEF_STRAIGHT_SPD    100.0f
#define DEF_SHIFT_RATIO     0.30f
#define DEF_STRAIGHT_VIEW   52.0f
#define DEF_ISLAND_SPD      60.0f
#define DEF_ISL_CFM         6
#define DEF_SPD_SLEW        8
#define DEF_SERVO_MID       6710
#define DEF_SERVO_LIM       1700
#define DEF_EXPOSURE        128
#define DEF_FF_GAIN         50.0f

/* 各参数的可调范围 */
#define PID_MAX_VAL         1000.0f
#define SPD_MAX_VAL         200.0f
#define SERVO_MID_MIN       3000.0f
#define SERVO_MID_MAX       9500.0f
#define SERVO_LIM_MAX       3000.0f
#define EXPOSURE_MIN        1.0f
#define EXPOSURE_MAX        2000.0f
#define DIFF_GAIN_MAX       3.0f
#define FF_GAIN_MAX         200.0f
#define SHIFT_RATIO_MAX     1.0f                        /* 变速系数上限, 1.0 = 视野最短时降到 0 */
#define STRAIGHT_VIEW_MAX   ((float)IMG_ROW)            /* 视野最长就是 IMG_ROW 行 */
#define ISL_CFM_MAX         12.0f                       /* 入环确认帧数上限 */
#define SPD_SLEW_MAX        30.0f                       /* 加速斜率上限(每帧最多涨多少) */

/* ================================================================ 菜单2 数据 */
#define PID_PARAM_NUM       9                   /* 9 个 PID 参数 */
#define PID_ITEM_NUM        10                  /* 9 个参数 + 1 个左右同步开关 */

static uint8    pid_cursor = 0;
static uint8    pid_sync   = 1;                 /* 1 = 左右电机 PID 一起改 */

static const char *const pid_name[PID_PARAM_NUM] =
{
    "SERVO KP", "SERVO KI", "SERVO KD",
    "LEFT  KP", "LEFT  KI", "LEFT  KD",
    "RIGHT KP", "RIGHT KI", "RIGHT KD"
};

/* ================================================================ 菜单3 数据 */
/* 参数表。★ 顺序不能随便动 ★
   par_get() / par_adjust() / menu_page3() 里都按下标硬编码,
   整型参数的下标范围(3~6)在 par_adjust 和 menu_page3 两处各写了一遍, 改顺序要一起改。 */
#define PAR_ITEM_NUM        12

#define PAR_INT_FIRST       3                   /* 第 3~9 项是整型(显示成整数、步长至少 1) */
#define PAR_INT_LAST        9

static uint8    par_cursor   = 0;
static int32    exposure_now = DEF_EXPOSURE;   /* 曝光值缓存, 改了才下发 */

static const char *const par_name[PAR_ITEM_NUM] =
{
    "BASE SPD",     /*  0 */  "STRAIGHT SPD",  /*  1 */  "SHIFT RATIO",   /*  2 */
    "STRAIGHT VIEW",/*  3 */  "ISLAND SPD",    /*  4 */  "SERVO MID",     /*  5 */
    "SERVO LIM",    /*  6 */  "EXPOSURE",      /*  7 */  "ISL CFM",       /*  8 */
    "SPD SLEW",     /*  9 */  "ACK GAIN",      /* 10 */  "FF GAIN"        /* 11 */
};

/* 菜单3 下面的状态行 —— 跟着参数条数走, 加了参数自动往下让。
   注: 丢图保护(WHITE/LL/LR/CNT)那两行已经被参数挤掉了 —— 保护本身照常工作,
       只是屏幕上不再显示它的调试数据。 */
#define M3_ROW_FPS          (PAR_ITEM_NUM + 1)  /* 13 */
#define M3_ROW_ELEM         (PAR_ITEM_NUM + 2)  /* 14  元素状态, 诊断直道加速/环岛用 */

/* ================================================================ 菜单4 数据 */
#define TST_ITEM_NUM        4

static uint8    tst_cursor = 0;

static const char *const tst_name[TST_ITEM_NUM] =
{
    "MOTOR TEST 2S", "RESET PID", "RESET PARAM", "LOST PROTECT"
};


/* ==========================================================================================
 *                                          显示辅助
 * ========================================================================================== */

/*-------------------------------------------------------------------------------------------------------------------
  @brief    显示一整行字符串, 自动用空格补齐到行宽, 把这行原来的内容覆盖掉
-------------------------------------------------------------------------------------------------------------------*/
static void menu_show_line(uint8 row, const char *str)
{
    char  buf[LINE_CHAR + 1];
    uint8 i = 0;

    while ((str[i] != '\0') && (i < LINE_CHAR))
    {
        buf[i] = str[i];
        i ++;
    }
    while (i < LINE_CHAR)
    {
        buf[i ++] = ' ';
    }
    buf[LINE_CHAR] = '\0';

    tft180_show_string(0, TEXT_ROW(row), buf);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    标题行: 左边标题, 右边靠右对齐显示一个附加信息(比如当前步长)
-------------------------------------------------------------------------------------------------------------------*/
static void menu_show_title(const char *left, const char *right)
{
    char  buf[LINE_CHAR + 1];
    uint8 i = 0, k = 0, rlen = 0;

    while ((right[rlen] != '\0') && (rlen < LINE_CHAR))
    {
        rlen ++;
    }

    while ((left[i] != '\0') && (i < LINE_CHAR))
    {
        buf[i] = left[i];
        i ++;
    }
    while (i < (LINE_CHAR - rlen))
    {
        buf[i ++] = ' ';
    }
    while ((k < rlen) && (i < LINE_CHAR))
    {
        buf[i ++] = right[k ++];
    }
    while (i < LINE_CHAR)
    {
        buf[i ++] = ' ';
    }
    buf[LINE_CHAR] = '\0';

    tft180_set_color(COL_TITLE_PEN, COL_TITLE_BG);
    tft180_show_string(0, TEXT_ROW(0), buf);
    tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    把 "> 参数名" 部分画出来, 整行先用底色刷一遍(顺带清掉旧内容)
  @note     返回时画笔颜色就是该行应该用的颜色, 调用方接着画数值即可
-------------------------------------------------------------------------------------------------------------------*/
static void menu_show_label(uint8 row, const char *name, uint8 selected)
{
    char  buf[LINE_CHAR + 1];
    uint8 i = 0;

    buf[i ++] = selected ? '>' : ' ';

    while ((*name != '\0') && (i < VAL_COL))
    {
        buf[i ++] = *name ++;
    }
    while (i < LINE_CHAR)
    {
        buf[i ++] = ' ';
    }
    buf[LINE_CHAR] = '\0';

    tft180_set_color(selected ? COL_SEL_PEN : COL_TEXT_PEN,
                     selected ? COL_SEL_BG  : COL_TEXT_BG);
    tft180_show_string(0, TEXT_ROW(row), buf);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    把浮点数按 3 位小数格式化成长度固定为 VAL_WIDTH 的字符串(右对齐)
  @note     不用库里的 tft180_show_float, 因为 func_double_to_str 是直接截断的:
            单精度下 0.7f 实际存的是 0.699999988, 乘 1000 取整会显示成 0.699。
            调参界面显示错数字很要命, 所以这里自己做四舍五入。
-------------------------------------------------------------------------------------------------------------------*/
static void menu_format_f(char *buf, float value)
{
    int32 scaled;
    int32 ip;
    int32 fp;
    uint8 i;
    uint8 neg = 0;

    /* ★ 支持负数 ★
       原来这里是 "value < 0 就归零", 因为当时所有可调参数都不允许负值。
       现在目标速度要支持倒车(负值), 所以改成: 先取出符号, 剩下的按绝对值格式化,
       符号画在最左边那一格 buf[0] —— 那一格本来是空着没用到的。
       VAL_WIDTH=9 刚好排得下: [符号][4位整数][.][3位小数] */
    if (value < 0.0f)
    {
        neg   = 1;
        value = -value;
    }
    if (value > 9999.999f)  value = 9999.999f;

    scaled = (int32)(value * 1000.0f + 0.5f);   /* 四舍五入到 0.001 */

    /* 数值小到四舍五入之后是 0 的, 不要显示成 "-0.000" */
    if (scaled == 0)
    {
        neg = 0;
    }

    ip = scaled / 1000;
    fp = scaled % 1000;

    for (i = 0; i < VAL_WIDTH; i ++)
    {
        buf[i] = ' ';
    }
    buf[VAL_WIDTH] = '\0';

    /* 小数部分固定 3 位 */
    buf[VAL_WIDTH - 1] = (char)('0' + (fp % 10));
    buf[VAL_WIDTH - 2] = (char)('0' + ((fp / 10) % 10));
    buf[VAL_WIDTH - 3] = (char)('0' + ((fp / 100) % 10));
    buf[VAL_WIDTH - 4] = '.';

    /* 整数部分最多 4 位, 不够的在左边留空格 */
    for (i = 0; i < 4; i ++)
    {
        buf[VAL_WIDTH - 5 - i] = (char)('0' + (ip % 10));
        ip /= 10;
        if (ip == 0)
        {
            break;
        }
    }

    /* 符号: 用最左边那个空位 */
    buf[0] = neg ? '-' : ' ';
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    可调参数行 —— 浮点值
-------------------------------------------------------------------------------------------------------------------*/
static void menu_show_param_f(uint8 row, const char *name, float value, uint8 selected)
{
    char buf[VAL_WIDTH + 1];

    menu_show_label(row, name, selected);
    menu_format_f(buf, value);
    tft180_show_string(VAL_X, TEXT_ROW(row), buf);
    tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    可调参数行 —— 整数值
-------------------------------------------------------------------------------------------------------------------*/
static void menu_show_param_i(uint8 row, const char *name, int32 value, uint8 selected)
{
    menu_show_label(row, name, selected);
    tft180_show_int(VAL_X, TEXT_ROW(row), value, VAL_INT_DIGIT);
    tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    可调参数行 —— 文本值(比如 ON / OFF)
-------------------------------------------------------------------------------------------------------------------*/
static void menu_show_param_s(uint8 row, const char *name, const char *value, uint8 selected)
{
    menu_show_label(row, name, selected);
    tft180_show_string(VAL_X, TEXT_ROW(row), value);
    tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    菜单4 的动作行, 没有数值
-------------------------------------------------------------------------------------------------------------------*/
static void menu_show_action(uint8 row, const char *name, uint8 selected)
{
    char  buf[LINE_CHAR + 1];
    uint8 i = 0;

    buf[i ++] = selected ? '>' : ' ';
    buf[i ++] = ' ';

    while ((*name != '\0') && (i < LINE_CHAR))
    {
        buf[i ++] = *name ++;
    }
    while (i < LINE_CHAR)
    {
        buf[i ++] = ' ';
    }
    buf[LINE_CHAR] = '\0';

    tft180_set_color(selected ? COL_SEL_PEN : COL_TEXT_PEN,
                     selected ? COL_SEL_BG  : COL_TEXT_BG);
    tft180_show_string(0, TEXT_ROW(row), buf);
    tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    菜单4 的按键电平监视, 0 表示按下
-------------------------------------------------------------------------------------------------------------------*/
static void menu_show_key_level(uint8 row)
{
    char  buf[LINE_CHAR + 1];
    uint8 lv[7];
    uint8 i;

    lv[0] = KEY1;  lv[1] = KEY2;  lv[2] = KEY3;  lv[3] = KEY4;
    lv[4] = KEY5;  lv[5] = KEY6;  lv[6] = KEY7;

    for (i = 0; i < LINE_CHAR; i ++)
    {
        buf[i] = ' ';
    }
    buf[LINE_CHAR] = '\0';

    for (i = 0; i < 7; i ++)
    {
        buf[1 + i * 2] = (lv[i] == 0) ? '0' : '1';
    }

    tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);
    tft180_show_string(0, TEXT_ROW(row), buf);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    菜单4 的状态行: GO 标志 + 左右轮当前速度
-------------------------------------------------------------------------------------------------------------------*/
static void menu_show_status(uint8 row)
{
    tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);
    menu_show_line(row, "GO:");

    tft180_show_int (3  * 6, TEXT_ROW(row), GO, 2);
    tft180_show_string(7 * 6, TEXT_ROW(row), "SPD L/R:");
    tft180_show_int (15 * 6, TEXT_ROW(row), g_left_speed, 4);
    tft180_show_char(20 * 6, TEXT_ROW(row), '/');
    tft180_show_int (21 * 6, TEXT_ROW(row), g_right_speed, 4);
}


/* ==========================================================================================
 *                                          菜单1  处理后的图像 + 实时数据
 *
 *  版面:
 *      (0,0) 起 94x60 显示连通域处理后的主道二值图, 上面叠边线和中线
 *      右边一窄列(x=96 起)放 8 行短参数
 *      底部 y=64 起放状态和提示
 * ========================================================================================== */

/* 图像按 IMG_COL x IMG_ROW 原尺寸 1:1 显示, 不做缩放, 这样看到的拐点才是真实质量 */
#define M1_IMG_X        0
#define M1_IMG_Y        0
#define M1_IMG_W        IMG_COL
#define M1_IMG_H        IMG_ROW

/* 右边短参数列: 左边 4 个字符放标签, 右边从第 24 像素起放 5 个字符的数值 */
#define M1_COL_X        (IMG_COL + 2)               /* 96 */
#define M1_VAL_X        (M1_COL_X + 4 * 6)          /* 120 */

/* ------------------------------------------------------------------------------------------
 *  ★ 刷屏优化: 静态标签和动态数值分开画 ★
 *
 *  为什么这么改:
 *    tft180_show_char() 每个字符要 6 次 set_region(每次约 11 字节 SPI 命令)
 *    + 48 次像素写入, 合计约 162 字节 SPI 和 54 次函数调用。
 *    菜单1 原来每帧画约 303 个字符, 其中 182 个是永远不变的标签 ——
 *    按 72MHz SPI 算, 光传输就 5ms 以上, 加上函数调用开销要 8~12ms,
 *    而 60fps 整帧预算才 16.7ms。
 *
 *  改法:
 *    标签只在"整页重画"(page_dirty)时画一次, 之后每帧只刷数值。
 *    数值每帧字符数约 120, 刷屏时间直接砍掉一多半。
 *
 *  安全性:
 *    tft180_show_int 是按固定位宽输出的(不足补空格), 所以数字位数变化不会留残影;
 *    标签和数值分处不同列(M1_COL_X / M1_VAL_X), 互不覆盖。
 * ------------------------------------------------------------------------------------------ */
#define MENU_TEXT_DIV       2               /* 数值每几帧刷一次, 1 = 每帧都刷 */

static void m1_label(uint8 row, const char *label)
{
    tft180_show_string(M1_COL_X, TEXT_ROW(row), label);
}

static void m1_val(uint8 row, int32 value)
{
    tft180_show_int(M1_VAL_X, TEXT_ROW(row), value, 4);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    菜单1
  @param    full  1 = 整页重画(画标签), 0 = 只刷数值
-------------------------------------------------------------------------------------------------------------------*/
static void menu_page1(uint8 full)
{
    static uint8 text_tick = 0;

    /* ---------- 图像每帧都刷(这是这一页的意义所在) ---------- */
    /* 阈值填 1: CDM 里只有 0/1, 0 画黑 1 画白 */
    tft180_show_gray_image(M1_IMG_X, M1_IMG_Y, BinaryImg_CDM[0],
                           IMG_COL, IMG_ROW, M1_IMG_W, M1_IMG_H, 1);
    TFT180_Show_Track_Lines(M1_IMG_X, M1_IMG_Y, M1_IMG_W, M1_IMG_H);

    tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);

    /* ---------- 静态标签: 只在换页时画一次 ----------
       注意: tft180_show_char 内部是 set_region(x, y, x, y+8), 也就是一个字实际占了
             y ~ y+8 共 9 行, 而 set_region 断言 y+8 < 128。
             所以本屏最多只能用到第 14 行 (14*8+8 = 120 < 128),
             第 15 行 (120+8 = 128) 会直接触发断言, 千万别用。 */
    if (full)
    {
        m1_label(0, "lsp:");
        m1_label(1, "rsp:");
        m1_label(2, "lpw:");
        m1_label(3, "rpw:");
        m1_label(4, "err:");
        m1_label(5, "ang:");
        m1_label(6, "thr:");
        tft180_show_string(M1_COL_X, TEXT_ROW(7), "GO: ");

        /* 十字和环岛都要看的三个量:
           BS = Boundry_Start_Left / Right, 边界起始行(越靠下越接近 IMG_ROW)
           B  = Both_Lost_Time, 双边同时丢线数
           SS = Search_Stop_Line, 最长白列长度(看到多远) */
        menu_show_line(8,  "BS:   /    B:    SS:");

        /* "R:" 原来是用 show_string 单独画的, 现在并进模板一起画 */
        menu_show_line(9,  "LOST L:     R:");

        /* 斑马线: ZB  = 状态 0待机 1通过中 2绕圈中(等第2次) 3第2次已越过 4已停车
                    SEE = 本帧是否看到斑马线
                    N   = 已经越过几次, 发车后应该是 0 -> 1 -> (跑一圈) -> 2 -> 停车
                    D   = 看不到斑马线后累计走的距离, 对照 ZEBRA_CONFIRM_DIST 标定
           ZB-R = 第1次越过后累计的距离, 对照 ZEBRA_REARM_DIST 标定 */
        menu_show_line(10, "ZB:  SEE:  N:  D:");

        /* 第 11 行: ZB-R 的数值占 5~10 列, FPS 的标签从第 12 列开始、数值从第 16 列开始。
           "ZB-R:" 是 5 个字符(0~4), 后面补 7 个空格(5~11), 然后 "FPS:" 正好落在 12~15 列。
           FPS 数值写在 16 列, 3 位数(show_int 的第 4 个参数就是位数)最多占 16~19 列, 不会越界。 */
        menu_show_line(11, "ZB-R:       FPS:");

        /* 环岛: ISL = 状态 0未识别 1~8 各个阶段 9刚跑完
                  LR  = L左环 / R右环 / -还没定
                  ANG = 偏航角(度)。本工程没有陀螺仪, 这个值是左右轮速差积分出来的,
                        标定 Island_Yaw_Gain 就靠看它。详见 island.c 顶部 */
        menu_show_line(12, "ISL:   LR:   ANG:");

        /* 十字四个拐点的行号 + Cross_Flag。
           斜入十字找不到角点时看 U 那一对; D 是下拐点行号, 它一跳补线斜率就跟着跳 */
        menu_show_line(13, "U:   /    D:   /    X:");

        /* 环岛入口判据的四个中间量:
             左环岛要求 MR==0 且 CL!=0 且 CR==0
             右环岛要求 ML==0 且 CL==0 且 CR!=1 */
        menu_show_line(14, "CL:   CR:   ML:   MR:");

        text_tick = MENU_TEXT_DIV;      /* 让整页重画这一帧也把数值画上 */
    }

    /* ---------- 动态数值 ----------
       降到 MENU_TEXT_DIV 帧画一次; 图像不受影响, 仍然每帧刷 */
    if (text_tick < MENU_TEXT_DIV)
    {
        text_tick ++;
        return;
    }
    text_tick = 0;

    /* 右边一列 */
    m1_val(0, Speed_Left_Real);
    m1_val(1, Speed_Right_Real);
    m1_val(2, g_left_pwm);
    m1_val(3, g_right_pwm);
    m1_val(4, (int32)Err);
    m1_val(5, (int32)angle);
    m1_val(6, (int32)ImgConf.Threshold);
    tft180_show_string(M1_VAL_X, TEXT_ROW(7), (GO == 1) ? "RUN" : "STOP");

    /* 底部状态 */
    tft180_show_int (3  * 6, TEXT_ROW(8), Boundry_Start_Left, 2);
    tft180_show_int (7  * 6, TEXT_ROW(8), Boundry_Start_Right, 2);
    tft180_show_int (13 * 6, TEXT_ROW(8), Both_Lost_Time, 2);
    tft180_show_int (20 * 6, TEXT_ROW(8), Search_Stop_Line, 2);

    tft180_show_int (7  * 6, TEXT_ROW(9), Left_Lost_Time, 3);
    tft180_show_int (14 * 6, TEXT_ROW(9), Right_Lost_Time, 3);

    tft180_show_int (3  * 6, TEXT_ROW(10), Zebra_Task_State, 1);
    tft180_show_int (9  * 6, TEXT_ROW(10), Zebra_Seen_Flag, 1);
    tft180_show_int (13 * 6, TEXT_ROW(10), Zebra_Pass_Cnt, 1);
    tft180_show_int (17 * 6, TEXT_ROW(10), (int32)Zebra_Lost_Dist, 5);

    tft180_show_int (5  * 6, TEXT_ROW(11), (int32)Zebra_Rearm_Dist, 5);
    tft180_show_int (16 * 6, TEXT_ROW(11), (int32)fps_val, 3);      /* 帧率 */

    tft180_show_int (4  * 6, TEXT_ROW(12), Island_State, 1);
    tft180_show_char(10 * 6, TEXT_ROW(12),
                     (Left_Island_Flag == 1) ? 'L' : ((Right_Island_Flag == 1) ? 'R' : '-'));
    tft180_show_int (17 * 6, TEXT_ROW(12), (int32)FJ_Angle, 4);

    tft180_show_int (2  * 6, TEXT_ROW(13), Left_Up_Find, 2);
    tft180_show_int (6  * 6, TEXT_ROW(13), Right_Up_Find, 2);
    tft180_show_int (12 * 6, TEXT_ROW(13), Left_Down_Find, 2);
    tft180_show_int (16 * 6, TEXT_ROW(13), Right_Down_Find, 2);
    tft180_show_int (22 * 6, TEXT_ROW(13), Cross_Flag, 1);

    tft180_show_int (3  * 6, TEXT_ROW(14), Island_Dbg_CL, 2);
    tft180_show_int (9  * 6, TEXT_ROW(14), Island_Dbg_CR, 2);
    tft180_show_int (15 * 6, TEXT_ROW(14), Island_Dbg_ML, 2);
    tft180_show_int (21 * 6, TEXT_ROW(14), Island_Dbg_MR, 2);
}


/* ==========================================================================================
 *                                          菜单2  PID 调参
 * ========================================================================================== */

/*-------------------------------------------------------------------------------------------------------------------
  @brief    按索引取 PID 参数地址
  @param    idx  0~2 舵机 kp/ki/kd, 3~5 左轮, 6~8 右轮
-------------------------------------------------------------------------------------------------------------------*/
static float *pid_value_ptr(uint8 idx)
{
    switch (idx)
    {
        case 0: return &servo_pid.kp;
        case 1: return &servo_pid.ki;
        case 2: return &servo_pid.kd;
        case 3: return &left_pid.kp;
        case 4: return &left_pid.ki;
        case 5: return &left_pid.kd;
        case 6: return &right_pid.kp;
        case 7: return &right_pid.ki;
        case 8: return &right_pid.kd;
        default: return 0;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    调整 PID 参数
  @param    idx   参数索引, 等于 PID_PARAM_NUM 时表示最后那个同步开关
            dir   +1 加, -1 减
-------------------------------------------------------------------------------------------------------------------*/
static void pid_adjust(uint8 idx, int8 dir)
{
    float *p;
    float  v;

    /* 最后一项是“左右同步”开关, 按加减就是切换状态 */
    if (idx >= PID_PARAM_NUM)
    {
        pid_sync = pid_sync ? 0 : 1;
        return;
    }

    p = pid_value_ptr(idx);
    if (p == 0)
    {
        return;
    }

    v = *p + step_tab[step_idx] * (float)dir;

    if (v < 0.0f)           v = 0.0f;
    if (v > PID_MAX_VAL)    v = PID_MAX_VAL;

    /* 归整到 3 位小数, 免得 0.1 反复累加攒出一堆浮点误差 */
    v = (float)((int32)(v * 1000.0f + 0.5f)) / 1000.0f;

    *p = v;

    /* 左右电机参数联动 */
    if (pid_sync)
    {
        float *q = 0;

        if ((idx >= 3) && (idx <= 5))
        {
            q = pid_value_ptr(idx + 3);
        }
        else if ((idx >= 6) && (idx <= 8))
        {
            q = pid_value_ptr(idx - 3);
        }

        if (q != 0)
        {
            *q = v;
        }
    }
}

static void menu2_key(uint8 key)
{
    switch (key)
    {
        case KEY_MIDDLE_PRES:                   /* KEY1 光标上移 */
            pid_cursor = (pid_cursor == 0) ? (PID_ITEM_NUM - 1) : (pid_cursor - 1);
            page_dirty = 1;
            break;

        case KEY_DOWN_PRES:
            pid_cursor ++;
            if (pid_cursor >= PID_ITEM_NUM)
            {
                pid_cursor = 0;
            }
            page_dirty = 1;
            break;

        case KEY_UP_PRES:                       /* KEY2 换步长 */
            step_idx ++;
            if (step_idx >= 3)
            {
                step_idx = 0;
            }
            page_dirty = 1;
            break;

        case KEY_RIGHT_PRES:
            pid_adjust(pid_cursor, +1);
            page_dirty = 1;
            break;

        case KEY_LEFT_PRES:
            pid_adjust(pid_cursor, -1);
            page_dirty = 1;
            break;

        default:
            break;
    }
}

static void menu_page2(void)
{
    uint8 i;

    tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);
    tft180_clear();

    menu_show_title("M2 PID TUNE", step_str[step_idx]);

    for (i = 0; i < PID_PARAM_NUM; i ++)
    {
        menu_show_param_f(1 + i, pid_name[i], *pid_value_ptr(i), (pid_cursor == i));
    }

    menu_show_param_s(1 + PID_PARAM_NUM, "SYNC L=R", pid_sync ? "ON" : "OFF",
                      (pid_cursor == PID_PARAM_NUM));
}


/* ==========================================================================================
 *                                          菜单3  运行参数
 * ========================================================================================== */

/*-------------------------------------------------------------------------------------------------------------------
  @brief    按索引读运行参数(统一按 float 处理)
-------------------------------------------------------------------------------------------------------------------*/
static float par_get(uint8 idx)
{
    switch (idx)
    {
        case 0:  return Base_Speed;                 /* 基准速度 */
        case 1:  return Straight_Speed;             /* 长直道速度 */
        case 2:  return Shift_Ratio;                /* 变速系数 */
        case 3:  return Straight_View;              /* 直道判定视野长度 */
        case 4:  return Island_Speed;               /* 环岛速度 */
        case 5:  return (float)medille;
        case 6:  return (float)servo_limit;
        case 7:  return (float)exposure_now;
        case 8:  return (float)Isl_Confirm_Frames;  /* 入环确认帧数 */
        case 9:  return (float)Spd_Slew_Up;         /* 加速斜率 */
        case 10: return ack_gain;                   /* 阿克曼增益(原 DIFF GAIN 槽位) */
        case 11: return speed_ff_gain;
        default: return 0.0f;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    调整运行参数
  @note     整型参数(中值/限幅/曝光)的步长至少是 1, 免得调出 6710.3 这种值
-------------------------------------------------------------------------------------------------------------------*/
static void par_adjust(uint8 idx, int8 dir)
{
    float step;
    float v;

    if (idx >= PAR_ITEM_NUM)
    {
        return;
    }

    step = step_tab[step_idx];
    if ((idx >= PAR_INT_FIRST) && (idx <= PAR_INT_LAST) && (step < 1.0f))
    {
        step = 1.0f;    /* 整型参数(视野长度/中值/限幅/曝光)步长至少 1 */
    }

    v = par_get(idx) + step * (float)dir;

    switch (idx)
    {
        case 0:     /* 基准速度 */
            if (v < 0.0f)           v = 0.0f;
            if (v > SPD_MAX_VAL)    v = SPD_MAX_VAL;
            Base_Speed = v;
            break;

        case 1:     /* 长直道速度 */
            if (v < 0.0f)           v = 0.0f;
            if (v > SPD_MAX_VAL)    v = SPD_MAX_VAL;
            Straight_Speed = v;
            break;

        case 2:     /* 变速系数 0~1 */
            if (v < 0.0f)               v = 0.0f;
            if (v > SHIFT_RATIO_MAX)    v = SHIFT_RATIO_MAX;
            Shift_Ratio = v;
            break;

        case 3:     /* 直道判定视野长度(行) */
            if (v < 0.0f)               v = 0.0f;
            if (v > STRAIGHT_VIEW_MAX)  v = STRAIGHT_VIEW_MAX;
            Straight_View = (float)(int32)v;    /* 取整, 免得出现 52.3 行 */
            break;

        case 4:     /* 环岛速度 */
            if (v < 0.0f)           v = 0.0f;
            if (v > SPD_MAX_VAL)    v = SPD_MAX_VAL;
            Island_Speed = (float)(int32)v;
            break;

        case 5:     /* 舵机中值 */
            if (v < SERVO_MID_MIN)  v = SERVO_MID_MIN;
            if (v > SERVO_MID_MAX)  v = SERVO_MID_MAX;
            medille = (int16)v;
            break;

        case 6:     /* 打舵限幅 */
            if (v < 0.0f)           v = 0.0f;
            if (v > SERVO_LIM_MAX)  v = SERVO_LIM_MAX;
            servo_limit = (int16)v;
            break;

        case 7:     /* 摄像头曝光 */
            if (v < EXPOSURE_MIN)   v = EXPOSURE_MIN;
            if (v > EXPOSURE_MAX)   v = EXPOSURE_MAX;
            exposure_now = (int32)v;
            mt9v03x_set_exposure_time((uint16)exposure_now);
            break;

        case 8:     /* 入环确认帧数 */
            if (v < 1.0f)                   v = 1.0f;
            if (v > ISL_CFM_MAX)            v = ISL_CFM_MAX;
            Isl_Confirm_Frames = (int)v;
            break;

        case 9:     /* 加速斜率 */
            if (v < 1.0f)                   v = 1.0f;
            if (v > SPD_SLEW_MAX)           v = SPD_SLEW_MAX;
            Spd_Slew_Up = (int)v;
            break;

        case 10:    /* 差速转弯增益 */
            if (v < 0.0f)           v = 0.0f;
            if (v > 2.0f)   v = 2.0f;       /* 上限 2.0: 超过 1.0 就是过度差速了 */
            v = (float)((int32)(v * 1000.0f + 0.5f)) / 1000.0f;
            ack_gain = v;
            break;

        case 11:    /* 速度前馈增益 */
            if (v < 0.0f)           v = 0.0f;
            if (v > FF_GAIN_MAX)    v = FF_GAIN_MAX;
            v = (float)((int32)(v * 1000.0f + 0.5f)) / 1000.0f;
            speed_ff_gain = v;
            break;

        default:
            break;
    }
}

static void menu3_key(uint8 key)
{
    switch (key)
    {
        case KEY_MIDDLE_PRES:                   /* KEY1 光标上移 */
            par_cursor = (par_cursor == 0) ? (PAR_ITEM_NUM - 1) : (par_cursor - 1);
            page_dirty = 1;
            break;

        case KEY_DOWN_PRES:
            par_cursor ++;
            if (par_cursor >= PAR_ITEM_NUM)
            {
                par_cursor = 0;
            }
            page_dirty = 1;
            break;

        case KEY_UP_PRES:                       /* KEY2 换步长 */
            step_idx ++;
            if (step_idx >= 3)
            {
                step_idx = 0;
            }
            page_dirty = 1;
            break;

        case KEY_RIGHT_PRES:
            par_adjust(par_cursor, +1);
            page_dirty = 1;
            break;

        case KEY_LEFT_PRES:
            par_adjust(par_cursor, -1);
            page_dirty = 1;
            break;

        default:
            break;
    }
}

static void menu_page3(void)
{
    uint8 i;

    tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);
    tft180_clear();

    menu_show_title("M3 RUN PARAM", step_str[step_idx]);

    for (i = 0; i < PAR_ITEM_NUM; i ++)
    {
        if ((i >= PAR_INT_FIRST) && (i <= PAR_INT_LAST))    /* 视野/中值/限幅/曝光是整型 */
        {
            menu_show_param_i(1 + i, par_name[i], (int32)par_get(i), (par_cursor == i));
        }
        else
        {
            menu_show_param_f(1 + i, par_name[i], par_get(i), (par_cursor == i));
        }
    }

    /* 参数占 1~12 行, 从第 13 行起放跑车时要盯的几个数。
       行号用 M3_ROW_xxx 宏, 加了参数会自动往下让。
       这里只画标签(和整页一起, 换页时画一次),
       后面的数值由 tft_show() 每帧刷新 —— 见 case 3 里的注释。

       SPD   = 本帧调度出来的目标速度(环岛 ISL>=2 期间会看到它掉到 ISLAND SPD)
       VIEW  = 最长白列长度(0~60), 也就是"有效视野", 标定 STRAIGHT VIEW 靠它
       ISL   = 环岛状态。>=2 时整段按 ISLAND SPD 跑, 也是"进不了环岛"的第一个排查点
       CROSS / ZB = 十字 / 斑马线标志 */
    /* 排版: FPS 值在 5~7 列, "SPD:" 在 9~12, SPD 值在 14~16, "VIEW:" 在 18~22, VIEW 值在 23~25 */
    menu_show_line(M3_ROW_FPS,   "FPS:     SPD:     VIEW:   ");

    /* 元素状态 */
    menu_show_line(M3_ROW_ELEM,  "ISL:    CROSS:    ZB:     ");
}


/* ==========================================================================================
 *                                          菜单4  测试 / 信息
 * ========================================================================================== */

static void menu4_exec(uint8 idx)
{
    switch (idx)
    {
        case 0:     /* 开环直行 2 秒, 原来是挂在 KEY7 上的自检 */
            Motor_Test_Start(2000);
            break;

        case 1:     /* 恢复 PID 默认值 */
            PID_init();
            break;

        case 2:     /* 恢复运行参数默认值 */
            Base_Speed     = DEF_BASE_SPD;
            Straight_Speed = DEF_STRAIGHT_SPD;
            Shift_Ratio    = DEF_SHIFT_RATIO;
            Straight_View  = DEF_STRAIGHT_VIEW;
            Island_Speed   = DEF_ISLAND_SPD;
            Isl_Confirm_Frames = DEF_ISL_CFM;
            Spd_Slew_Up        = DEF_SPD_SLEW;
            medille      = DEF_SERVO_MID;
            servo_limit  = DEF_SERVO_LIM;
            exposure_now = DEF_EXPOSURE;
            diff_gain    = 0.0f;    /* 旧差速保持关闭 */
            ack_gain     = 1.0f;    /* 阿克曼恢复到完整比例 */
            speed_ff_gain = DEF_FF_GAIN;
            mt9v03x_set_exposure_time((uint16)exposure_now);
            break;

        case 3:     /* 丢图保护 开/关 */
            Img_Lost_On   = (Img_Lost_On == 0) ? 1 : 0;
            Img_Lost_Cnt  = 0;      /* 切换时把连续计数清掉, 免得刚打开就停车 */
            Img_Lost_Flag = 0;
            break;

        default:
            break;
    }
}

static void menu4_key(uint8 key)
{
    switch (key)
    {
        case KEY_MIDDLE_PRES:                   /* KEY1 光标上移 */
            tst_cursor = (tst_cursor == 0) ? (TST_ITEM_NUM - 1) : (tst_cursor - 1);
            page_dirty = 1;
            break;

        case KEY_DOWN_PRES:
            tst_cursor ++;
            if (tst_cursor >= TST_ITEM_NUM)
            {
                tst_cursor = 0;
            }
            page_dirty = 1;
            break;

        case KEY_UP_PRES:                       /* KEY2 执行选中项 */
            menu4_exec(tst_cursor);
            page_dirty = 1;
            break;

        default:
            break;
    }
}

static void menu_page4(void)
{
    uint8 i;

    if (page_dirty)
    {
        page_dirty = 0;

        tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);
        tft180_clear();

        /* 菜单4 没有步长可调, 标题右边就不挂了 */
        menu_show_title("M4 TEST/INFO", "");

        for (i = 0; i < TST_ITEM_NUM; i ++)
        {
            menu_show_action(1 + i, tst_name[i], (tst_cursor == i));
        }

        /* 第 4 项是开关而不是动作, 右边把当前状态写出来。
           KEY2 执行时会清 page_dirty, 所以切换后这行会跟着重画 */
        tft180_show_string(19 * 6, TEXT_ROW(4),
                           (Img_Lost_On != 0) ? "ON " : "OFF");

        menu_show_line(8,  "KEY LEVEL 1-7:");
        menu_show_line(10, "(0 = PRESSED)");

        /* ---- 环岛状态1 诊断 (锁存值, 数值在下面每帧刷) ----
           专门用来查"状态1 的补线为什么没画出来"。

           为什么不在菜单1 看 ML/MR:
               那两个量在每个摄像头帧的开头都会被状态0 的判据结果覆盖,
               只有"这一帧正好是状态1"时才显示状态1 的搜索结果。
               屏幕刷新跟帧率同步, 状态1 又可能只有几帧, 所以基本抓不到。
               下面这组是锁存快照, 跑完一圈停下来看就行。

           L / R = 状态1 单调突变点的搜索结果。0 = 没找到(补线不画)
           CNT   = 进过状态1 的总帧数。0 = 从头到尾没进过状态1
           SS    = 那一刻的最长白列(Search_Stop_Line)
           TOP   = 扫描范围上界。>= 15 说明整个范围被 isl_clamp_range 夹没了,
                   搜索结果必然是 0, 补线必然不画 */
        menu_show_line(5,  "S1 BAND 7..15 OWN SIDE");
        menu_show_line(7,  "S1L:     S1R:     HIT:    ");
        menu_show_line(12, "DUR:     SS:      TOP:   ");
        menu_show_line(13, "LR:      BS:      LT:    ");
        menu_show_line(14, "WHY:     1LOST2RNG3FLT4PR");
    }

    /* 这两行是实时变化的, 每帧都刷 */
    menu_show_key_level(9);
    menu_show_status(11);

    /* 环岛状态1 诊断的数值。9 个 show_int 一共 27 个字符,
       比菜单1 那张图便宜得多, 每帧刷没有负担。
       列位置对准 menu_page4 里三条模板的空档:
           行12 -> 5/14/23, 行13 -> 7/16/23, 行14 -> 5/14/23
       (每个值 2 位数, show_int 实际画 3 个字符, 上面都留了 3 列空档) */
    /* 行6: 搜索窗那 9 行的边界原值。每格 2 字符 = 12 像素, 9 格共 108 像素, 一行放得下。
       第 1 格是第 7 行, 第 9 格是第 15 行 */
    {
        uint8 bi;
        for (bi = 0; bi < ISL_BAND_N; bi ++)
        {
            tft180_show_int((uint16)(bi * 12), TEXT_ROW(6), Isl_Diag_S1_BAND[bi], 1);
        }
    }

    tft180_show_int(5  * 6, TEXT_ROW(7),  Isl_Diag_S1_L,   2);
    tft180_show_int(14 * 6, TEXT_ROW(7),  Isl_Diag_S1_R,   2);
    tft180_show_int(23 * 6, TEXT_ROW(7),  Isl_Diag_S1_HIT, 2);

    tft180_show_int(5  * 6, TEXT_ROW(12), Isl_Diag_S1_DUR, 2);
    tft180_show_int(14 * 6, TEXT_ROW(12), Isl_Diag_S1_SS,  2);
    tft180_show_int(23 * 6, TEXT_ROW(12), Isl_Diag_S1_TOP, 2);

    tft180_show_int(5  * 6, TEXT_ROW(13), Isl_Diag_S1_LR,  2);
    tft180_show_int(14 * 6, TEXT_ROW(13), Isl_Diag_S1_BS,  2);
    tft180_show_int(23 * 6, TEXT_ROW(13), Isl_Diag_S1_LT,  2);

    tft180_show_int(5  * 6, TEXT_ROW(14), Isl_Diag_S1_WHY, 2);
}


/* ==========================================================================================
 *                                          菜单5  直道标准赛宽标定
 *
 *  用法
 *      1. 把车放在直道上, 车头方向平行于赛道边线(车摆正), 保持静止
 *      2. 按 KEY2 开始采集(连续 32 帧求平均, 约 0.5 秒)
 *      3. 屏幕上直接显示成 C 数组, 手抄下来即可
 *
 *  版面 (16 行 x 26 字符)
 *      行 0     标题
 *      行 1~2   操作提示
 *      行 3     状态: OK 有效行数 / RNG 最大波动 / ST 采集状态
 *      行 4     base_width[60]={
 *      行 5~12  数组内容, 每行 8 个值, 每个值固定占 3 字符(2 位数 + 逗号)
 *      行 13    };
 *      行 14    KEY7 提示
 *
 *      RNG(最大波动) 是 32 帧里同一行宽度的 (最大-最小), 单位像素。
 *      它很小(比如 0~2)说明边界很稳, 标定结果可信;
 *      某几行如果特别大, 说明那几行的边界本来就抓不稳, 后续用的时候要留个心眼。
 * ========================================================================================== */

#define WCAL_PER_LINE       8                   /* 一行显示 8 个值 x 3 字符 = 24 <= 26 */
#define WCAL_FIRST_ROW      5                   /* 数组第一行画在第 5 行 */
#define WCAL_DATA_ROWS      8                   /* 8 行 x 8 个 = 64 >= IMG_ROW(60) */

static uint8 wcal_dirty      = 1;               /* 数组区需要重画 */
static uint8 wcal_last_state = 0xFF;            /* 上一次的采集状态, 用来检测变化 */

/*-------------------------------------------------------------------------------------------------------------------
  @brief    画数组的一行, 每行 WCAL_PER_LINE 个值
  @param    start  本行第一个值的下标(行号)
  @note     每个值固定占 3 个字符: 2 位数字 + 逗号。不足 2 位的左边补空格,
            这样不管宽度是 1 位数还是 2 位数, 后面所有值的列位置都不动
-------------------------------------------------------------------------------------------------------------------*/
static void wcal_show_data(uint8 row, uint8 start)
{
    char  buf[LINE_CHAR + 1];
    uint8 i;

    for (i = 0; i < LINE_CHAR; i ++)
    {
        buf[i] = ' ';
    }
    buf[LINE_CHAR] = '\0';

    for (i = 0; i < WCAL_PER_LINE; i ++)
    {
        uint8 idx = (uint8)(start + i);
        uint8 p   = (uint8)(i * 3);
        int   v;

        if (idx >= IMG_ROW)
        {
            break;                              /* 数组放完了 */
        }

        v = Base_Width[idx];
        if (v > 99)
        {
            v = 99;                             /* 宽度不可能超过 IMG_COL, 这里只是防越界 */
        }

        if (v >= 10)
        {
            buf[p] = (char)('0' + (v / 10));
        }
        buf[p + 1] = (char)('0' + (v % 10));
        buf[p + 2] = ',';
    }

    tft180_show_string(0, TEXT_ROW(row), buf);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    状态行: 有效行数 / 最大波动 / 采集状态
  @note     列位置和下面那句 menu_show_line 里的模板严格对齐:
            tft180_show_int 的 num 是"保留低 num 位", 实际会占 num+1 个字符
-------------------------------------------------------------------------------------------------------------------*/
static void wcal_show_status(uint8 row)
{
    const char *st;

    tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);

    /*              0         1         2
     *              01234567890123456789012345  */
    menu_show_line(row, "OK:   /   RNG:   ST:");
    tft180_show_int (3  * 6, TEXT_ROW(row), IMG_ROW - Base_Width_Bad, 2);   /* 列 3~5  */
    tft180_show_int (7  * 6, TEXT_ROW(row), IMG_ROW, 2);                    /* 列 7~9  */
    tft180_show_int (14 * 6, TEXT_ROW(row), Base_Width_MaxRng, 2);          /* 列 14~16 */

    /* 状态字固定 6 个字符, 把 CAP -> DONE 切换时可能留下的残字一起盖掉 */
    switch (Width_Calib_State)
    {
        case 1:  st = "CAP   ";  break;
        case 2:  st = "DONE  ";  break;
        default: st = "IDLE  ";  break;
    }
    tft180_show_string(20 * 6, TEXT_ROW(row), st);

    if (Width_Calib_State == 1)
    {
        tft180_show_int(23 * 6, TEXT_ROW(row), Width_Calib_Prog, 2);        /* 列 23~24 */
    }
}

static void menu5_key(uint8 key)
{
    switch (key)
    {
        case KEY_UP_PRES:                       /* KEY2 开始采集 */
            Width_Calib_Start();
            page_dirty = 1;
            break;

        case KEY_LEFT_PRES:                     /* KEY5 清空结果 */
            Width_Calib_Clear();
            page_dirty = 1;
            break;

        default:
            break;
    }
}

static void menu_page5(void)
{
    uint8 i;

    /* 采集状态一变(开始 / 完成)就把数组区重画一次 */
    if (wcal_last_state != Width_Calib_State)
    {
        wcal_last_state = Width_Calib_State;
        wcal_dirty      = 1;
    }

    if (page_dirty)
    {
        page_dirty = 0;
        wcal_dirty = 1;

        tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);
        tft180_clear();

        menu_show_title("M5 WIDTH CAL", "");

        menu_show_line(1,  "CAR ON STRAIGHT, STILL");
        menu_show_line(4,  "base_width[60]={");
        menu_show_line(13, "};");
    }

    /* 数组区只在内容变了才重画, 平时不刷, 免得闪 */
    if (wcal_dirty)
    {
        wcal_dirty = 0;

        for (i = 0; i < WCAL_DATA_ROWS; i ++)
        {
            wcal_show_data((uint8)(WCAL_FIRST_ROW + i), (uint8)(i * WCAL_PER_LINE));
        }
    }

    wcal_show_status(3);        /* 采集进度要动, 每帧都刷 */
}


/* ==========================================================================================
 *                                          菜单6  空白页(跑车专用)
 *
 *  用途: 比赛/跑圈时切到这一页, 屏幕就完全不干活了。
 *
 *  为什么需要它:
 *      菜单1 要每帧推一整幅灰度图 + 边界线 + 一堆文字, 实测能把帧率从 50fps 拖到 25fps。
 *      而帧率直接决定控制延迟 —— 延迟又直接决定"高速能不能进环岛"这类事。
 *      跑车时屏幕上那些数其实没人看, 所以留一个"什么都不画"的页面。
 *
 *  这一页的行为:
 *      换页进来时清一次屏 + 画两行静态说明, 之后【每帧一个像素都不画】。
 *      所以切到这一页, 帧率就是这套硬件能达到的上限。
 *
 *  用 KEY6 翻页切到它; 想记帧率就翻回菜单3。
 * ========================================================================================== */
static void menu_page6(void)
{
    if (page_dirty)
    {
        page_dirty = 0;

        tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);
        tft180_clear();

        /* 只有换页这一帧画, 之后就再也不碰屏幕了 */
        menu_show_line(0, "M6 BLANK");
        menu_show_line(2, "SCREEN IDLE - NO DRAW");
        menu_show_line(4, "USE THIS PAGE WHEN RUNNING");    /* 26 字符, 再多会被截断 */
        menu_show_line(6, "FPS IS HIGHEST HERE");
    }

    /* ★ 这里故意什么都不画 ★
       想在这儿看帧率的话加一行 tft180_show_int(...) 也行,
       但那就不是"零开销"了, 跑车时建议还是翻回菜单3 看。 */
}


/* ==========================================================================================
 *                                          菜单主入口
 * ========================================================================================== */
void tft_show(void)
{
    uint8 key = Key_Get();

    /* 每进一次 tft_show 就是一个摄像头帧, 帧率统计放在最前面, 保证每个页面都在数 */
    fps_tick();

    /* ---------------- 按键分发 ---------------- */
    if (key == KEY6_PRES)
    {
        /* 切换菜单 */
        menu_page ++;
        if (menu_page > MENU_PAGE_NUM)
        {
            menu_page = 1;
        }
        page_dirty = 1;
    }
    else if (key == KEY7_PRES)
    {
        /* 启动 / 停止, 任何页面都有效 */
        GO = -GO;
        page_dirty = 1;
    }
    else if (key != 0)
    {
        switch (menu_page)
        {
            case 2: menu2_key(key); break;
            case 3: menu3_key(key); break;
            case 4: menu4_key(key); break;
            case 5: menu5_key(key); break;
            default: break;
        }
    }

    /* ---------------- 电机测试到点自动停 ---------------- */
    Motor_Test_Tick();

    /* ---------------- 画面 ---------------- */
    switch (menu_page)
    {
        case 1:
        {
            /* full = 1 时整页重画(清屏 + 画静态标签), 否则只刷数值。
               注意要先取走 page_dirty 再清零, 否则 menu_page1 就看不到这次是不是整页了。
               图像会盖住大半个屏幕, 所以进页面时那一次清屏是必须的, 用来擦上一页的残字 */
            uint8 full = page_dirty;

            if (page_dirty)
            {
                page_dirty = 0;
                tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);
                tft180_clear();
            }
            menu_page1(full);
            break;
        }

        case 2:
            if (page_dirty)
            {
                page_dirty = 0;
                menu_page2();
            }
            break;

        case 3:
            if (page_dirty)
            {
                page_dirty = 0;
                menu_page3();       /* 整页重画, 包含下面几行的标签 */
            }

            /* ★ 跑车时看的页面 ★
               这一页平时一帧都不重画, 所以在这里补几个实时数值 —— 每帧十几二十个字符,
               约 3KB SPI, 对帧率的影响可以忽略, 所以帧率读数基本就是真实值。
               想看"刷屏吃掉了多少帧率", 就把它和菜单1 第 11 行那个数比一比。 */
            tft180_set_color(COL_TEXT_PEN, COL_TEXT_BG);

            /* 帧率 / 当前调度速度 / 有效视野
               ★ 列位置要按"画 num+1 个字符"来算, 不是 num 个 ★
                  tft180_show_int 内部是先 memset(num+1 个空格) 再让 func_int_to_str
                  往里写数字, 而 func_int_to_str 不写结束符 —— 所以剩下的空格也一起画出来了。
                   阈值 x + 6*num + 5 < 160, 也就是 x + 6*num <= 154
                   列 5  num=3 -> x=30,  30+18=48  ✓
                   列14  num=3 -> x=84,  84+18=102 ✓
                   列23  num=2 -> x=138, 138+12=150 ✓  (原来写的列24 -> 156, 会顶到 161 触发断言) */
            tft180_show_int(5  * 6, TEXT_ROW(M3_ROW_FPS), (int32)fps_val, 3);
            tft180_show_int(14 * 6, TEXT_ROW(M3_ROW_FPS), (int32)g_sched_speed, 3);
            tft180_show_int(23 * 6, TEXT_ROW(M3_ROW_FPS), Search_Stop_Line, 2);

            /* 元素状态: 环岛(速度调度的闸) / 十字 / 斑马线 */
            tft180_show_int(5  * 6, TEXT_ROW(M3_ROW_ELEM), Island_State, 1);
            tft180_show_int(15 * 6, TEXT_ROW(M3_ROW_ELEM), Cross_Flag, 1);
            tft180_show_int(22 * 6, TEXT_ROW(M3_ROW_ELEM), Zebra_Seen_Flag, 1);
            break;

        case 4:
            menu_page4();       /* 内部自己处理 page_dirty */
            break;

        case 5:
            menu_page5();       /* 内部自己处理 page_dirty */
            break;

        case 6:
            menu_page6();       /* 空白页: 只有换页那一帧画, 之后每帧零开销 */
            break;

        default:
            menu_page = 1;
            page_dirty = 1;
            break;
    }
}
