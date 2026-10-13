#ifndef _WHEEL_H_
#define _WHEEL_H_
#include "zf_common_headfile.h"
#include "Ourcode_headfile.h"
//ye
//电机频率
#define WHEEL_FRE 25000

//设置轮子 切换编号可以交换左右轮
//#define WHEEL_LEFT 1    //左轮编号
//#define WHEEL_RIGH 0    //右轮编号
//设置正方向 切换正负1调整正方向
//#define LEFT_FRONT (-1)
//#define RIGH_FRONT (1)
//PWM比较值最大值
#define WHEEL_MAX 9999

extern int16 spd;
extern int16 target_encoder;
extern float D_speed,left_speed, right_speed;
extern float left_pwm;
extern float right_pwm;

extern volatile int g_left_pwm;
extern volatile int g_right_pwm;
extern volatile int g_left_speed;
extern volatile int g_right_speed;
extern volatile int g_left_increment;
extern volatile int g_right_increment;

//左右轮闭环目标速度。★ 不再是手动设的 ★
//由 Speed_Schedule_Update() 每帧按视野长度自动算出来, 差速再往上叠
extern float target_lsp;
extern float target_rsp;

//速度调度参数(Base_Speed/Straight_Speed/Shift_Ratio/Straight_View)
//定义在 control.c, 声明在 control.h, 菜单3 可调
extern volatile float g_sched_speed;    //本帧调度出来的速度, 菜单显示用
extern volatile int   Spd_Slew_Up;      //加速斜率(每帧最多涨多少), 菜单3 的 "SPD SLEW"

//按视野长度算目标速度, 每个摄像头帧调用一次(必须在 Longest_White_Column 之后)
void Speed_Schedule_Update(void);

//差速转弯增益(旧方案: 差速 = 视觉误差 × 增益)。已被阿克曼取代, 默认 0
extern float diff_gain;

/* ---- 阿克曼差速 ----
   原理: 转弯时内外轮绕同一圆心画不同半径的圆, 速度必须和半径成正比,
         否则轮子拖滑、码盘测到的速度就不是车实际的速度。
         差速比例  k = (d/L)·tan|δ|,   v内 = v(1-k),  v外 = v(1+k)
   详见 wheel.c 里的完整推导。 */
extern float ack_gain;       //阿克曼增益, 菜单3的 "ACK GAIN" 可调。0=关, 1=完整阿克曼
extern float ack_ratio;      //d/L = 后轮半轮距 / 轴距, 必须实测
extern float ack_delta_max;  //舵机打满时前轮的实际转角(度), 必须实测

//速度前馈增益(目标速度 -> 基础PWM)，菜单3可调。需开环标定后填写
extern float speed_ff_gain;

//速度环复位，起步时调用
void Velocity_Start(void);

//1ms 系统计时，在 TIM5 中断里累加
extern volatile uint32 sys_ms;

//开环电机测试（菜单4用）
extern volatile uint8 Motor_Test_Flag;

//轮初始化
void wheel_init(void);
//设置轮速  最大为正负WHEEL_MAX
void wheel_ctrl(int lspeed,int rspeed);
void stop_start(void);

void Motor_Test_Start(uint16 ms);
void Motor_Test_Tick(void);


#endif
