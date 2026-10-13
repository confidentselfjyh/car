#include "zf_common_headfile.h"
#include "Control.h"
#include "math.h"
#include "visionpid.h"

#define VALID_HEIGHT  15  //只扫描图像最下面的15行，需要改扫描行数就改这个值

//电磁/摄像头标志位，默认是摄像头
volatile uint8 Electromagnet_Flag=0;//0=摄像头，1=电磁
extern volatile uint8 Stop_Flag;

//图像信息
extern volatile int Search_Stop_Line;     //搜索截止行
extern volatile int Left_Line[IMG_ROW];  //左边界数组
extern volatile int Right_Line[IMG_ROW]; //右边界数组

//发车与停止相关参数
volatile uint8 Go=8;             //发车阶段的标志位，默认不发车
volatile uint8 Speed_Mode=0;     //速度模式，0=慢，1=中，2=快
volatile uint8 Go_Direction=1;   //0=正跑，1=反跑（倒车）
extern volatile uint8 Straight_Flag;//直道标志位

//转向相关
int Steer_Angle=0;        //舵机角度值
volatile float Err=0;     //摄像头误差
volatile extern float ADC_Err;//电磁误差

//速度环
int ADC_Speed=180;     //电磁速度
int Speed_Left_Set=300;
int Speed_Right_Set=300;
int Ramp_Speed=200;    //坡道速度
int Barricade_Speed=250;//路障段速度，防止直道加速冲过去
float Island_Diff=0.4;  //环岛偏差
float Err_Diff=1.1;     //误差放大系数，加速过弯时防止车头扫线
float ADC_Diff=0.4;     //电磁偏差
float Island_Ratio=0;   //环岛比例

/* ================================ 速度调度参数 ================================
 *  这三个(加一个判定阈值)是 Speed_Schedule_Update() 用的, 菜单3 可调。
 *  原工程这里留的是电磁车的值(Base=330 / Straight=400 / Shift=1.5), 单位对不上
 *  (本车是编码器计数/10ms, 50 就差不多 1m/s), 所以数值重新给了。
 *
 *      Base_Speed       基准速度。视野正常时跑这个 —— 大部分时间是它
 *      Straight_Speed   长直道速度。视野够远 = 前方是长直道, 直接冲
 *      Shift_Ratio      变速系数 0~1。视野越短(弯道)在基准速度上砍得越狠:
 *                           0.0 -> 永远 Base_Speed(等于关掉变速)
 *                           0.5 -> 视野最短时降到 Base_Speed 的一半
 *                        (原工程写的是 1.5, 那是另一套算法的系数, 别照搬)
 *      Straight_View    视野达到多少行就认为前方是长直道
 * ============================================================================ */
float Base_Speed     = 150.0f;   //基准速度
float Straight_Speed = 165.0f;  //直道速度
float Shift_Ratio    = 0.30f;   //变速系数 0~1
float Straight_View  = 52.0f;   //直道判定的视野长度(行)

/* 环岛速度。菜单3 可调。
 *
 *  触发条件: Island_State >= 2 时整段按这个速度跑, 回到 0/1 就交还给常规调度。
 *
 *  ★ 为什么是 >=2 而不是"一离开 0 就减速" ★
 *    状态1 只是"刚认定这是环岛", 车往往离入口还有一段(实测状态1 判得偏早)。
 *    如果状态1 就开始慢, 车会在进环前的那段路上白白爬很久。
 *    状态2 是"下拐点丢了、上方出现弧线", 这时候才是真的快到口上了。
 *
 *  为什么不能只用上面的变速逻辑:
 *    环岛入口最大的麻烦是"车冲太快 —— 等状态机认出来的时候已经过站了"。
 *    光"取消直道加速"不够, 必须把速度实打实压下来。
 *
 *  注意: 万一环岛状态机卡在 >=2 的状态出不来, 车会一直用这个速度跑 ——
 *        这属于"安全的失败方向"(慢而不是快), 但屏幕上 ISL 会一直 >=2, 一眼能看出来。 */
float Island_Speed   = 150.0f;   //环岛速度

//环岛相关变量
extern volatile int Island_State;     //环岛状态标志
extern volatile int Ramp_Flag;        //坡道标志

//路障相关
extern volatile uint8 Barricade_Flag; //路障标志
extern uint8 Obstacle_Dir; //0=右拐，1=左拐

//权重数组
const uint8 Weight[MT9V03X_H]=
{
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端00 ——09 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端10 ——19 行权重
        1, 1, 1, 1, 1, 1, 1, 3, 4, 5,              //图像最远端20 ——29 行权重
        6, 7, 9,11,13,15,17,19,20,20,              //图像最远端30 ——39 行权重
       19,17,15,13,11, 9, 7, 5, 3, 1,              //图像最远端40 ——49 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端50 ——59 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端60 ——69 行权重
};


/* 注: 原来这里还有个 Err_Sum(), 内容和 Err_Sum1() 是同一套加权平均,
       但带着两个毛病(循环下界会到 -1 越界读 / weight_count 可能是 0 除以零),
       而且整个工程从没调用过。已经删掉, 只留 Err_Sum1() 这一份。 */


/*-------------------------------------------------------------------------------------------------------------------
  @brief     摄像头误差获取 —— 加权平均法
  @return    加权平均后的误差。
             正 = 赛道中线在图像中心左边 = 车偏右了 = 需要左转
             (和 Err_Sum3 的符号完全一致)
  @note      ★ 这一版换成了"加权平均"算法 ★

             做法: 从图像最下面一行(离车最近)往上扫, 一直扫到搜索截止行,
                   把每一行的偏差按 Weight[] 给的权重加权、再求平均。

             为什么要加权:
               权重大 = "控制前瞻行", 相当于告诉 PID "主要看这几行"。
               权重集中在离车中等距离的那几行 -> 车切内;
               权重靠下(离车近) -> 反应慢、切外。
               权重表 Weight[] 就在本文件上面, 想改前瞻位置直接改它。

             ★ 切换前必读 ★
               旧的 Err_Sum1 算的是 "中线 - 图像中心", 符号和 Err_Sum3 是【反的】。
               如果直接把 main.c 切到旧版 Err_Sum1, 舵机会往反方向打, 一发车就冲出去。
               这一版已经统一成 "图像中心 - 中线", 和 Err_Sum3 一致,
               所以现在切过来是安全的, 不需要改任何别的代码。
-------------------------------------------------------------------------------------------------------------------*/
float Err_Sum1(void)
{
    int   i;
    int   bottom_row;
    int   top_row;

    float err          = 0.0f;
    float weight_count = 0.0f;

    /* 从图像最下面一行(离车最近)往上扫 */
    bottom_row = IMG_ROW - 1;

    /* 扫到"搜索截止行"为止。Search_Stop_Line 就是最长白列的长度 */
    top_row = IMG_ROW - Search_Stop_Line - 1;

    /* ★ 下界兜底(修原算法的数组越界) ★
       原来的写法是 i >= IMG_ROW - Search_Stop_Line - 1。
       Search_Stop_Line 满值正好等于 IMG_ROW(60) 时, 这个下界算出来是 -1,
       循环会去读 Weight[-1] / Left_Line[-1] / Right_Line[-1], 越界。
       而直道上最长白列经常就是满值, 所以这个越界是常态, 必须夹住。 */
    if (top_row < 0)
    {
        top_row = 0;
    }

    for (i = bottom_row; i >= top_row; i --)
    {
        /* 图像中心 - 左右边界的中点 = 这一行的偏差
           >>1 等效于除以 2 */
        err          += (float)(IMG_COL / 2 - ((Left_Line[i] + Right_Line[i]) >> 1)) * (float)Weight[i];
        weight_count += (float)Weight[i];
    }

    /* ★ 除零兜底 ★
       Search_Stop_Line 为 0(整幅图都看不到赛道)时, 上面一次循环都不进,
       weight_count 是 0, 直接相除会得到 nan。
       nan 喂给 PID 会让舵机乱打, 所以这里退回 0。 */
    if (weight_count <= 0.0f)
    {
        return 0.0f;
    }

    return err / weight_count;
}
  
 
 
#define VALID_HEIGHT  15  //只扫描图像最下面的15行，需要改扫描行数就改这个值
 
float Err_Sum()
{
    int i=0;
    float sum=0;
    for(i=IMG_ROW-1;i>IMG_ROW-VALID_HEIGHT-1;i--)
    {
        sum+=((IMG_COL - 1) * 0.5f)-Mid_Line[i];
    }
    return sum/15.0f;
}
