#include "wheel.h"
#include "visionpid.h"

int16 spd = 2500;
int16 target_encoder = -500;
float D_speed = 0.0,left_speed = 0.0, right_speed = 0.0;

 //轮初始化
void wheel_init(void)
{
    //pwm_init(TIM8_PWM_CH1_C6, WHEEL_FRE,1000);
    //pwm_init(TIM8_PWM_CH2_C7, WHEEL_FRE,1000);
    //pwm_init(TIM4_PWM_CH2_D13, WHEEL_FRE,1000);
    //pwm_init(TIM4_PWM_CH3_D14, WHEEL_FRE,1000);
    pwm_init(MOTOR01_SPEED_PIN, WHEEL_FRE, 0);          //左电机 PWM
    pwm_init(MOTOR02_SPEED_PIN, WHEEL_FRE, 0);          //右电机 PWM
    gpio_init(MOTOR01_DIR_PIN, GPO, 1, GPIO_PIN_CONFIG);    //左电机方向, 上电默认正转
    gpio_init(MOTOR02_DIR_PIN, GPO, 1, GPIO_PIN_CONFIG);    //右电机方向, 上电默认正转
}
//内部函数，按照编号控制电机
/*static void wheel_pwm_out(uint8 wheel_num,int dir,int speed)

{
    int temp_speed=speed*dir;
    temp_speed =_limit((-PWM_DUTY_MAX),(temp_speed), (PWM_DUTY_MAX));

    if(wheel_num)
    {
        pwm_set_duty(TIM4_PWM_MAP1_CH2_D13,3600+temp_speed);
    }
    else
    {
        pwm_set_duty(TIM4_PWM_MAP1_CH3_D14,3600+temp_speed);
    }
}
*/



/* ================================ 电机输出(带方向) ================================
 *
 *  硬件是"方向脚 + PWM"方案:
 *      MOTOR01_DIR_PIN (D12) / MOTOR02_DIR_PIN (D15)   0 = 反转, 1 = 正转
 *      MOTOR01_SPEED_PIN (D13) / MOTOR02_SPEED_PIN (D14)   占空比 0 ~ MOTOR_PWM_MAX
 *
 *  ★ 为什么要加这一段 ★
 *  加反转之前, 速度环的输出被夹在 [0, 6000], 电机只能正转、不能制动, 于是:
 *      向前推轮子 -> 实际速度>0 -> 误差为负 -> PID 想输出负 PWM 制动 -> 被夹成 0
 *                    -> 电机断电自由滑行 -> 手感上"没阻尼"
 *      向后推轮子 -> 实际速度为负 -> 误差为正 -> PID 输出正 PWM -> 电机顶着你转
 *                    -> 手感上"有阻尼"
 *  两个方向本来是对称的, 只因为负的那一半被砍掉了才变成那样。
 *
 *  真正的危害不在"推轮子"这个实验, 而在跑车时: 减速、下坡、被推、差速转弯的内侧轮,
 *  全都只能靠摩擦滑行, 没法主动减速。
 * ================================================================================= */

/* 目标速度的绝对值上限(编码器计数/10ms)。菜单里本来就有 SPD_MAX_VAL=200 限制,
   这里只是再加一道兜底, 免得差速叠加之后把目标推飞 —— 不是"可达速度"的上限。
   真正能跑多快由电机输出决定: 前馈 ff = 目标 × speed_ff_gain, 输出又被夹在
   [-MOTOR_PWM_REV_MAX, MOTOR_PWM_MAX], 所以能跟得上的目标上限是
   MOTOR_PWM_MAX / speed_ff_gain (增益 50 时约 120)。目标给得比这还大,
      只是让输出一直顶在限幅上而已, 不会更快。 */
#define MOTOR_TGT_MAX        200.0f

#define MOTOR_PWM_DEADZONE   150    /* 死区: |speed| 小于它就只关 PWM。
                                       小占空比本来就推不动电机, 但会让方向脚在 0 附近
                                       反复翻转 —— DIR 每翻一次 H 桥都要走一遍死区时间,
                                       白费电、发热, 还会有"滋滋"声。
                                       稳态 PWM 在 2500 量级, 150 完全不影响控制精度。 */

/*-------------------------------------------------------------------------------------------------------------------
  @brief     单个电机的底层输出: 按符号决定方向脚, 按绝对值决定占空比
  @param     dir_pin   方向脚   spd_pin  PWM 通道   speed  带符号的占空比
-------------------------------------------------------------------------------------------------------------------*/
static void motor_out(gpio_pin_enum dir_pin, pwm_channel_enum spd_pin, int speed)
{
    if (speed > MOTOR_PWM_DEADZONE)
    {
        gpio_set_level(dir_pin, 1);                     /* 正转 */
        pwm_set_duty(spd_pin, (uint32)speed);
    }
    else if (speed < -MOTOR_PWM_DEADZONE)
    {
        gpio_set_level(dir_pin, 0);                     /* 反转(制动) */
        pwm_set_duty(spd_pin, (uint32)(-speed));
    }
    else
    {
        /* 死区内: 只把 PWM 关掉, 方向脚保持不动。
           零占空比下方向脚是什么电平都无所谓, 不动它反而少翻转一次 */
        pwm_set_duty(spd_pin, 0);
    }
}

/*设置轮速
 * lspeed / rspeed：带符号的占空比, 负数 = 反转(制动)
 *                 范围 -MOTOR_PWM_MAX ~ +MOTOR_PWM_MAX
 */
void wheel_ctrl(int lspeed,int rspeed)
{
    motor_out(MOTOR01_DIR_PIN, MOTOR01_SPEED_PIN, lspeed);   //左
    motor_out(MOTOR02_DIR_PIN, MOTOR02_SPEED_PIN, rspeed);   //右
}

void stop_start(void)
{
    wheel_ctrl(spd, spd + 100);
    //left_speed  = InrcPid(&left_pid, target_encoder, encoder_get(0));//左
    //right_speed = InrcPid(&right_pid, target_encoder, encoder_get(1));//右
    //wheel_ctrl(_limit((spd - left_speed),0,10000), _limit((spd - right_speed),0,10000));
}
// void Velocity_Control(int speed_left_real,int speed_right_real)
// {
//     static int target_lsp=70,target_rsp=70;
//     static int lsp,rsp;
//     lsp=InrcPid(&left_pid,speed_left_real,target_lsp);
//     rsp=InrcPid(&right_pid,speed_right_real,target_rsp);
//     if (lsp>=7000){lsp=7000;}if(lsp<=0){lsp=0;}
//     if (rsp>=7000){rsp=7000;}if(rsp<=0){rsp=0;}
//     pwm_set_duty(TIM4_PWM_MAP1_CH2_D13,lsp);//左
//     pwm_set_duty(TIM4_PWM_MAP1_CH3_D14,rsp);//右
// }

#define MOTOR_PWM_MAX     6000.0f   /* 正转(前进)最大占空比 */
#define MOTOR_PWM_REV_MAX 3000.0f   /* 反转(制动)最大占空比。
                                       制动太猛会把轮子抱死打滑, 高速下反而更危险,
                                       所以先只给一半。觉得制动力不够再往上加, 上限 MOTOR_PWM_MAX。
                                       注意: 这一项只是"允许的制动强度", 不影响正常前进。 */
/* 注: 原来这里有个固定的 MOTOR_CORR_MAX(±3000) 用来防积分饱和。
        加了反转之后它会把制动力卡死(见 Velocity_Control 里的说明),
        已经改成"跟着前馈走"的动态夹子, 所以这个宏删掉了。 */

/*-------------------------------------------------------------------------------------------------------------------
  速度前馈增益: 目标速度 -> 基础 PWM。

  没有前馈时, 输出完全靠积分从 0 往上爬, 以 kp=8/ki=0.3 为例每 10ms 只涨 15,
  爬到能推动车的 2500 要 1.4 秒 —— 这就是起步慢的原因。

  加了前馈之后, PWM 一起步就在正确量级, PID 只负责修那一点点残差。

  这个值需要开环标定: 例如目标速度 50 对应的稳态 PWM 是 2500, 这里就填 50。
  菜单3可在线调。
-------------------------------------------------------------------------------------------------------------------*/
float speed_ff_gain = 50.0f;

volatile int g_left_pwm  = 0;
volatile int g_right_pwm = 0;

volatile int g_left_speed  = 0;
volatile int g_right_speed = 0;

volatile int g_left_increment  = 0;
volatile int g_right_increment = 0;

/* 左右轮闭环目标速度。★ 不再是手动设的 ★
   现在由下面的 Speed_Schedule_Update() 每帧按视野长度自动算出来,
   差速(diff_l/diff_r)再在这个基础上叠加。 */
float target_lsp = 50.0f;
float target_rsp = 50.0f;

/* ================================ 速度调度 ================================
 *
 *  车不该一直用一个速度跑: 直道要冲, 弯道要收。
 *  这里按"有效视野长度"(Search_Stop_Line, 也就是最长白列的长度)自动算这一帧跑多快。
 *
 *  三个参数(菜单3可调):
 *      Base_Speed       基准速度。视野好的时候就用它 —— 大部分时间跑这个速度
 *      Straight_Speed   长直道速度。视野足够远 = 前方是长直道, 直接冲
 *      Shift_Ratio      变速系数 0~1。视野越短(弯道)在基准速度上砍得越狠:
 *                          0.0 -> 永远是 Base_Speed (等于关掉变速)
 *                          0.5 -> 视野最短时降到 Base_Speed 的一半
 *                          1.0 -> 视野最短时降到 0
 *
 *  算法:
 *      ratio = 视野长度 / IMG_ROW          (0 = 看不到, 1 = 看到最远)
 *      speed = Base_Speed * (1 - Shift_Ratio * (1 - ratio))
 *      视野 >= Straight_View 时, 往 Straight_Speed 过渡
 *
 *  为什么直道速度要做"过渡带"而不是直接跳:
 *      视野刚够到阈值的那一帧, 速度如果直接从 Base 跳到 Straight, 车轮会"顿"一下;
 *      直道上本来就跑得快, 这一下很容易打滑。所以留了 SPD_BLEND_ROWS 行的过渡带,
 *      视野每多一行就多要一点速度, 到 阈值+8 行 时才完全用 Straight_Speed。
 *
 *  注意: Search_Stop_Line 是在 Longest_White_Column() 里算出来的,
 *        所以本函数必须在它之后调用(现在是挂在 Zebra_Task_Tick 里, 见那边的注释)。
 * ====================================================================== */

/* 注: Base_Speed / Straight_Speed / Shift_Ratio / Straight_View 四个参数
       定义在 control.c(菜单3 可调), 这里只是使用。 */

/* 直道速度的过渡带宽度(行)。
   ★ 这个值不能大 ★
   视野(View)最大也就 IMG_ROW=60, 而真正的直道上大概只有 53~57。
   原来给 8 行, 配上默认阈值 52, 意味着要 View 顶到 60 才拿到完整的 Straight_Speed ——
   实际赛道上永远达不到, 所以看起来"直道还是 base 速度"。
   改成 4 行: 阈值+4 行就吃满直道速度。 */
#define SPD_BLEND_ROWS   4

/* ================================ 斜率限制 ================================
 *
 *  为什么需要:
 *    Search_Stop_Line 在"由弯入直 / 由直入弯"的那一两帧会突然跳一大截
 *    (赛道远端一消失, 最长白列可能从 60 直接掉到 40)。
 *    速度要是跟着一跳, 就是一脚急加速或者一脚急刹 —— 车会"卡一下"。
 *
 *  做法: 算出来的目标速度不允许一帧变太多, 按每帧最多变多少来爬。
 *        50fps 下一帧 20ms, SPD_SLEW_UP=3 就是每秒最多涨 150。
 *
 *  单位: 编码器计数/10ms(和 Base_Speed 同一个单位), 每"帧"允许的变化量。
 *
 *  注意: 只作用在常规调度上。环岛那一段(Island_State != 0)是直接返回的,
 *        不吃这个限制 —— 因为进环必须"认出来就立刻慢下来", 慢一步就过站了。
 *        反过来, 出环(状态回 0)的第一帧 g_sched_speed 还是 Island_Speed,
 *        会从这里平滑地爬上去, 出环反而更顺。
 * ====================================================================== */
volatile int Spd_Slew_Up = 6;           /* 每帧最多涨多少(加速斜率)。菜单3 的 "SPD SLEW" 就是改这个 */

/* 减速斜率 = 加速斜率 × 这个比例。
   降得比涨快一点更安全(该慢的时候要慢得下来), 但没必要单独给一个菜单项, 一个旋钮够了。 */
#define SPD_SLEW_DOWN_RATIO  1.6f

volatile float g_sched_speed = 0.0f;    /* 本帧调度出来的速度, 菜单3显示用 */

/*-------------------------------------------------------------------------------------------------------------------
  @brief     按视野长度算出这一帧的目标速度
  @note      每个摄像头帧调用一次, 必须在 Longest_White_Column() 之后
-------------------------------------------------------------------------------------------------------------------*/
void Speed_Schedule_Update(void)
{
    float ratio;
    float speed;
    float t;
    float slew_up;
    float slew_down;

    /* ★ 环岛优先, 直接返回 ★
     *
     *  条件: Island_State >= 2。从状态2 开始整段按 Island_Speed 跑,
     *        一直到状态回到 0 或 1 才交还给下面的常规调度。
     *
     *  ★ 为什么不是"一离开 0 就减速" ★
     *    状态1 只是"刚认定这是环岛", 这时车离入口往往还有一段(状态1 判得偏早)。
     *    从状态1 就慢的话, 车会在进环前那段路上白白爬很久。
     *    状态2 = "下拐点丢了、上方开始出现弧线", 这时候才是真的快到口上了。
     *
     *  为什么要在这里就 return, 而不是只把"直道加速"关掉:
     *    环岛入口的麻烦是"车冲太快 —— 等状态机认出来已经过站了"。
     *    只取消加速不够, 必须把速度实打实压下去。
     *    而且返回之后连变速那一项也不算了, 环岛里就是干净的一个定值, 好排查。 */
    if (Island_State >= 2)
    {
        speed = Island_Speed;
        if (speed < 0.0f) {speed = 0.0f;}

        g_sched_speed = speed;
        target_lsp = speed;
        target_rsp = speed;
        return;
    }

    /* ---- 以下是常规调度(走到这里说明 Island_State == 0) ---- */

    /* 视野比例: 0 = 完全看不到, 1 = 看到最远(IMG_ROW 行) */
    ratio = (float)Search_Stop_Line / (float)IMG_ROW;
    if (ratio > 1.0f) {ratio = 1.0f;}
    if (ratio < 0.0f) {ratio = 0.0f;}

    /* 基础变速: 视野满的时候正好是 Base_Speed, 视野越短砍得越多 */
    speed = Base_Speed * (1.0f - Shift_Ratio * (1.0f - ratio));

    /* 长直道: 从 Straight_View 开始, 在 SPD_BLEND_ROWS 行内过渡到 Straight_Speed。
     *
     * 注: 环岛的情况已经在函数开头 return 掉了, 走到这里 Island_State 一定是 0,
     *     所以不用再判环岛。十字不用管: 十字的视野本来就长, 而且十字通常在直道上,
     *     冲过去是对的。 */
    if ((float)Search_Stop_Line >= Straight_View)
    {
        t = ((float)Search_Stop_Line - Straight_View) / (float)SPD_BLEND_ROWS;
        if (t > 1.0f) {t = 1.0f;}

        speed = speed + (Straight_Speed - speed) * t;
    }

    if (speed < 0.0f) {speed = 0.0f;}

    /* ---- 斜率限制: 不许一帧变太多, 免得弯直过渡时车"卡一下" ----
       g_sched_speed 是上一帧真正发出去的速度, 拿它当基准。
       开机第一帧 g_sched_speed=0, 会从 0 慢慢爬上去 —— 这正好等于软启动。 */
    slew_up   = (float)Spd_Slew_Up;
    slew_down = slew_up * SPD_SLEW_DOWN_RATIO;

    if (speed > (g_sched_speed + slew_up))   {speed = g_sched_speed + slew_up;}
    if (speed < (g_sched_speed - slew_down)) {speed = g_sched_speed - slew_down;}
    if (speed < 0.0f) {speed = 0.0f;}

    g_sched_speed = speed;

    /* 左右轮的基础目标速度 = 调度速度(差速再往上叠) */
    target_lsp = speed;
    target_rsp = speed;
}

/* ================================ 差速转弯 ================================
 * 舵机负责转向，后轮差速帮忙: 转弯时内侧轮减速、外侧轮加速，能明显减小转弯半径，
 * 也能减轻舵机的负担。
 *
 * 符号约定(不能搞反):
 *     Err = 图像中心 - 赛道中线
 *     Err > 0  <=>  赛道中线在图像中心左侧  <=>  车偏右了  <=>  需要左转
 *     左转时内侧是左轮 => 左轮减速, 右轮加速
 * ======================================================================== */
/* ★ 差速增益 —— 现在默认 0(关闭) ★
   阿克曼差速(ack_gain)已经接管了转弯时的差速, 这一个是被它取代的旧方案:
   "差速量 = 视觉误差 × 增益", 和舵机打舵量没有定量关系。
   想两个叠加就把这里改回 2.0 —— 但先单独跑一遍阿克曼看效果, 不然分不清是谁的功劳。
   注: 菜单3 上这个槽位已经让给 "ACK GAIN" 了, 现在只能在代码里改。 */
float diff_gain = 0.0f;             /* 旧差速增益, 0 = 关闭 */

#define DIFF_LIMIT_RATIO   0.6f     /* 差速上限 = 基础速度的60%, 保证内侧轮不会被减到0以下 */
#define DIFF_FILTER        0.30f    /* Err 一阶滤波系数, 越小越平滑但越滞后 */

static float diff_l = 0.0f;         /* 左轮差速分量 */
static float diff_r = 0.0f;         /* 右轮差速分量 */

static float left_corr  = 0.0f;     /* 左轮 PID 修正量(叠加在前馈之上) */
static float right_corr = 0.0f;     /* 右轮 PID 修正量 */

extern volatile float Err;          /* 定义在 control.c */
extern volatile float angle;        /* 舵机指令, 定义在 main.c。阿克曼差速要用它 */
extern int16 servo_limit;           /* 舵机打舵限幅, 定义在 servo.c */

/* ================================ 阿克曼差速 ================================
 *
 *  原理
 *  ------------------------------------------------------------------------
 *  车转弯的时候, 四个轮子是绕【同一个圆心】画圆的(阿克曼几何)。后轴中心
 *  画半径 R 的圆, 内侧后轮画 R-d, 外侧后轮画 R+d, 其中 d 是后轮半轮距。
 *
 *  要让三个轮子都"纯滚动不打滑", 速度必须和各自的半径成正比:
 *
 *          v内 / v外 = (R - d) / (R + d)
 *
 *  而 R = L / tan|δ|  (L = 轴距, δ = 前轮转角), 代进去整理一下:
 *
 *          k = (d / L) * tan|δ|          <- 内外轮的速度差比例
 *          v内 = v * (1 - k)             v外 = v * (1 + k)
 *
 *  直线时 δ=0 => k=0 => 两轮同速, 自动退化成普通驱动。
 *
 *  为什么写成 k 而不是先算 R:
 *      δ→0 时 R→∞, 直接算会溢出/掉精度; 而 tan|δ| 有界, 天然安全。
 *
 *  ------------------------------------------------------------------------
 *  不做这个会怎样
 *  ------------------------------------------------------------------------
 *  转弯时内外轮必须走不同的距离。如果硬让它们同速, 轮子就会【拖滑】,
 *  而码盘测的是【轮子转了多少圈】, 不是【车走了多远】。
 *  于是速度环拿到的是错的反馈 —— 它以为车在以某个速度走, 其实不是。
 *  表现为: 弯道里速度环忽快忽慢地修, 车一顿一顿的; 转弯半径也不准。
 *
 *  ------------------------------------------------------------------------
 *  为什么这叫"和舵机协调"
 *  ------------------------------------------------------------------------
 *  差速量是从【舵机的实际指令】算出来的 —— 舵机打多少度, 差速就跟多少,
 *  两者由同一套几何绑死。原来的 diff_l/diff_r 是 Err 乘一个拍出来的增益,
 *  和舵机打舵量没有定量关系, 舵机换了参数它也不跟着变。
 * ======================================================================== */

float ack_gain      = 1.0f;     /* 阿克曼增益, 菜单3 的 "ACK GAIN" 可调。
                                   0 = 关闭; 1.0 = 完整阿克曼(理论值, 推荐);
                                   >1 = 过度差速, 外侧轮反而会打滑, 一般不推荐 */
float ack_ratio     = 0.39f;    /* d/L = 后轮半轮距 / 轴距。★ 必须实测, 见调试说明 ★ */
float ack_delta_max = 35.0f;    /* 舵机打满(±servo_limit)时, 前轮的实际转角(度)。
                                   ★ 也必须实测 ★ */

#define ACK_INNER_MIN   0.45f   /* 内侧轮速度最低保留比例。
                                   防止极低速 + 大转角时把内侧轮减到接近 0 */
#define ACK_DEG2RAD     0.0174532925f

static float ack_l = 1.0f;      /* 左轮的阿克曼比例因子, 1.0 = 不修正 */
static float ack_r = 1.0f;

/*-------------------------------------------------------------------------------------------------------------------
  @brief     按舵机指令算出左右轮的阿克曼比例因子 ack_l / ack_r
  @note      每 10ms(和速度环同周期)调用一次。
             符号约定(已从 visionpid.c 推导确认):
                 Err = 图像中心 - 赛道中线, Err>0 = 车偏右 = 要左转
                 LocaPid 里 error_curr = target - input = 0 - Err = -Err, 且 kp>0
                 => Err>0 时 angle < 0
                 所以: angle < 0 = 左转, angle > 0 = 右转
                 (和 servo.c 的 duty = medille - angle 也自洽: 左转 duty 变大)
-------------------------------------------------------------------------------------------------------------------*/
static void ackermann_update(void)
{
    float delta;        /* 前轮转角, 度, 左正右负 */
    float a;            /* |δ| 的弧度值 */
    float k;
    float lim;

    ack_l = 1.0f;
    ack_r = 1.0f;

    if (ack_gain <= 0.0f)       { return; }     /* 关了 */
    if (ack_delta_max <= 0.0f)  { return; }     /* 还没标定 */
    if (servo_limit <= 0)       { return; }     /* 防除零 */

    /* 舵机指令 -> 前轮转角。取负是因为 angle<0 才是左转 */
    delta = -(angle / (float)servo_limit) * ack_delta_max;

    /* tan|δ|, 用泰勒展开 tan(x) ≈ x + x³/3 (δ≤30° 时误差 <1%),
       这样不用引 math.h, 也省掉一次三角函数调用 */
    a = delta;
    if (a < 0.0f) { a = -a; }
    a = a * ACK_DEG2RAD;
    a = a + (a * a * a) * 0.3333333f;

    k = ack_ratio * a * ack_gain;               /* 内外轮速度差比例 */

    lim = 1.0f - ACK_INNER_MIN;
    if (k > lim) { k = lim; }
    if (k < 0.0f) { k = 0.0f; }

    if (delta > 0.0f)                           /* 左转: 左轮是内轮, 慢 */
    {
        ack_l = 1.0f - k;
        ack_r = 1.0f + k;
    }
    else                                        /* 右转: 右轮是内轮, 慢 */
    {
        ack_l = 1.0f + k;
        ack_r = 1.0f - k;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     按视觉误差计算左右轮的差速分量
  @note      Err 每帧才更新一次, 直接拿来用会让差速跟着中线抖, 所以先做一阶滤波。
             由 Velocity_Control() 每 10ms 调用一次。
-------------------------------------------------------------------------------------------------------------------*/
static void differential_update(void)
{
    static float err_filt = 0.0f;
    float base;
    float limit;
    float d;

    /* ★ 阿克曼差速: 每周期都算, 而且必须在下面那个"倒车直接返回"之前 ★
       它是纯几何前馈, 正着走倒着走都成立(倒车时 v 为负, 内外比的符号自动
       跟着对), 不应该和旧差速一起被倒车逻辑关掉。 */
    ackermann_update();

    /* 一阶低通滤波 */
    err_filt = err_filt * (1.0f - DIFF_FILTER) + Err * DIFF_FILTER;

    /* 上限跟着基础速度走, 这样改目标速度时差速量自动跟着缩放 */
    base = (target_lsp + target_rsp) * 0.5f;

    /* ★ 倒车(目标为负)时不做差速 ★
     *
     *  必须在这里就返回, 有两个原因:
     *    1. 转向几何整个反过来了 —— 倒车时"车偏右要左转"这个关系不再成立,
     *       差速照着原来的符号去帮, 只会和舵机打架。倒车就是用来测电机方向的,
     *       让两个轮子干净地一起反转最好。
     *    2. 就算不管语义, 数学上也是错的: base 是负数时 limit = base*0.6 也是负数,
     *       下面那两个夹子 (d > limit) 和 (d < -limit) 会互相打架 ——
     *       比如 limit=-30、d=8, 第一句把 d 夹成 -30, 第二句又把它夹成 +30,
     *       最后输出一个完全错误的差速值。 */
    if (base < 0.0f)
    {
        diff_l = 0.0f;
        diff_r = 0.0f;
        return;
    }

    limit = base * DIFF_LIMIT_RATIO;

    d = err_filt * diff_gain;

    if (d >  limit) d =  limit;
    if (d < -limit) d = -limit;

    diff_l = -d;    /* Err>0 要左转, 左轮(内侧)减速 */
    diff_r =  d;    /* 右轮(外侧)加速 */
}

void Velocity_Control(int speed_left_real, int speed_right_real)
{
    float left_increment;
    float right_increment;
    float tgt_l;
    float tgt_r;
    float ff_l;
    float ff_r;
    float left_pwm;
    float right_pwm;

    /* 先算差速, 再拿差速后的目标速度去做闭环 */
    differential_update();

    /* ★ 两条差速在这里合流 ★
       ack_* 是"乘性"的 —— 按转弯半径把基础速度按比例分配(阿克曼),
       diff_* 是"加性"的 —— 旧方案, 按视觉误差额外拱一点(默认已关)。
       ack_l/ack_r 在直线时正好是 1.0, 所以不开阿克曼时这一行和原来完全一样。 */
    tgt_l = target_lsp * ack_l + diff_l;
    tgt_r = target_rsp * ack_r + diff_r;

    /* ★ 下界从 0 改成了 -MOTOR_TGT_MAX ★
       原来是 tgt<0 就归零, 那样负的目标速度根本传不到下面去, 倒车就无从谈起。
       现在允许负值, 负目标会依次变成: 负前馈 -> 负输出 -> 方向脚拉低 + 正占空比。
       上界也一并夹一下, 防止差速把目标推到离谱的值。 */
    if (tgt_l >  MOTOR_TGT_MAX) {tgt_l =  MOTOR_TGT_MAX;}
    if (tgt_l < -MOTOR_TGT_MAX) {tgt_l = -MOTOR_TGT_MAX;}
    if (tgt_r >  MOTOR_TGT_MAX) {tgt_r =  MOTOR_TGT_MAX;}
    if (tgt_r < -MOTOR_TGT_MAX) {tgt_r = -MOTOR_TGT_MAX;}

    /* ---- 前馈: 目标速度直接映射成基础 PWM, 一起步就在正确量级 ---- */
    ff_l = tgt_l * speed_ff_gain;
    ff_r = tgt_r * speed_ff_gain;

    /* ---- 反馈: 增量式 PID 只负责修正前馈的残差 ---- */
    left_increment  = InrcPid(&left_pid,  (float)speed_left_real,  tgt_l);
    right_increment = InrcPid(&right_pid, (float)speed_right_real, tgt_r);

    left_corr  += left_increment;
    right_corr += right_increment;

    /* ★ 抗卷绕: 修正量的上下限跟着前馈走 ★
     *
     *  原来是固定夹在 ±MOTOR_CORR_MAX(3000)。加了反转之后这个夹子就成了瓶颈:
     *  稳态时前馈 ff=2500, 修正量最多 -3000, 于是最大制动只有 2500-3000 = -500,
     *  "允许反转"等于开了个寂寞。
     *
     *  改成让"前馈 + 修正"最终必然落在电机输出范围内:
     *          ff + corr ∈ [-MOTOR_PWM_REV_MAX, +MOTOR_PWM_MAX]
     *  即
     *          corr ∈ [-MOTOR_PWM_REV_MAX - ff, +MOTOR_PWM_MAX - ff]
     *
     *  好处有两个:
     *    1. 高速(ff 大)时自动允许更大的制动修正, 低速时自动收紧, 不用手调;
     *    2. 输出一旦顶到限幅, 修正量就正好停在"刚好顶住"的位置, 不会继续偷偷积分,
     *       扰动一消失立刻能退回来 —— 这比固定夹子更不容易卷绕。 */
    if (left_corr  > ( MOTOR_PWM_MAX     - ff_l)) {left_corr  =  MOTOR_PWM_MAX     - ff_l;}
    if (left_corr  < (-MOTOR_PWM_REV_MAX - ff_l)) {left_corr  = -MOTOR_PWM_REV_MAX - ff_l;}
    if (right_corr > ( MOTOR_PWM_MAX     - ff_r)) {right_corr =  MOTOR_PWM_MAX     - ff_r;}
    if (right_corr < (-MOTOR_PWM_REV_MAX - ff_r)) {right_corr = -MOTOR_PWM_REV_MAX - ff_r;}

    /* ---- 输出 = 前馈 + 反馈修正 ---- */
    left_pwm  = ff_l + left_corr;
    right_pwm = ff_r + right_corr;

    /* ★ 下界从 0 改成了 -MOTOR_PWM_REV_MAX ★
       原来是 [0, 6000], 负数一律夹成 0 —— 意味着"永远不制动"。
       现在允许负值, 由 wheel_ctrl() 翻译成"方向脚拉低 + 正占空比"。
       上面提到过的现象(向前推没阻尼、向后推有阻尼)就是这一行造成的。 */
    if (left_pwm > MOTOR_PWM_MAX)          {left_pwm = MOTOR_PWM_MAX;}
    else if (left_pwm < -MOTOR_PWM_REV_MAX){left_pwm = -MOTOR_PWM_REV_MAX;}
    if (right_pwm > MOTOR_PWM_MAX)          {right_pwm = MOTOR_PWM_MAX;}
    else if (right_pwm < -MOTOR_PWM_REV_MAX){right_pwm = -MOTOR_PWM_REV_MAX;}

    g_left_pwm  = (int)left_pwm;
    g_right_pwm = (int)right_pwm;

    g_left_speed  = speed_left_real;
    g_right_speed = speed_right_real;

    g_left_increment  = (int)left_increment;
    g_right_increment = (int)right_increment;

    wheel_ctrl(g_left_pwm, g_right_pwm);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     速度环复位, 起步(GO 由 0 变 1)时调用
  @note      前馈是每周期实时算的, 不用在这里重置;
             这里只清 PID 修正量和历史, 免得上一段跑动的残留造成重新起步时一次大增量。
-------------------------------------------------------------------------------------------------------------------*/
void Velocity_Start(void)
{
    left_corr  = 0.0f;
    right_corr = 0.0f;

    left_pid.error_curr  = 0.0f;
    left_pid.error_last  = 0.0f;
    left_pid.error_pre   = 0.0f;

    right_pid.error_curr = 0.0f;
    right_pid.error_last = 0.0f;
    right_pid.error_pre  = 0.0f;
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     开环电机测试（菜单4用）
  @param     ms          持续毫秒数，到点自动停
  @note      直接用固定占空比驱动两个电机，不经过编码器闭环。
             测试期间 TIM6 中断会跳过车轮控制，把 PWM 让给这里。
-------------------------------------------------------------------------------------------------------------------*/
#define MOTOR_TEST_PWM      1500

volatile uint8 Motor_Test_Flag = 0;
static   uint32 motor_test_end = 0;

void Motor_Test_Start(uint16 ms)
{
    Motor_Test_Flag = 1;
    motor_test_end  = sys_ms + ms;
    wheel_ctrl(MOTOR_TEST_PWM, MOTOR_TEST_PWM);
}

void Motor_Test_Tick(void)
{
    if (Motor_Test_Flag && ((int32)(sys_ms - motor_test_end) >= 0))
    {
        Motor_Test_Flag = 0;
        wheel_ctrl(0, 0);
    }
}

