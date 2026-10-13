#ifndef IMG_PROCESS
#define IMG_PROCESS

/* 注意: 正主在 cam_preprocess.h, 这里必须和那边保持一致 */
#define	IMG_ROW		60
#define	IMG_COL		94

void Longest_White_Column(void);

/* ---- 图像处理中间结果, 按“行”索引(每行一个值) ---- */
extern const uint8 Image_Flags[][9][8];     //数字图上的数字表(本工程未使用)
extern volatile int Left_Line[IMG_ROW];     //每行的左边界列坐标
extern volatile int Right_Line[IMG_ROW];    //每行的右边界列坐标
extern volatile int Mid_Line[IMG_ROW];      //每行的中线列坐标
extern volatile int Road_Wide[IMG_ROW];     //每行的赛道宽度
extern volatile int White_Column[IMG_COL];  //每列从底部往上连续白点的个数(注意按“列”索引)
extern volatile int Search_Stop_Line;       //搜索截止行,记录的是最长白列的长度,用的时候要用 IMG_ROW 减掉
extern volatile int Boundry_Start_Left;     //左边界起始行
extern volatile int Boundry_Start_Right;    //右边界起始行
extern volatile int Left_Lost_Time;         //左边界连续丢线的行数
extern volatile int Right_Lost_Time;        //右边界连续丢线的行数
extern volatile int Both_Lost_Time;         //两侧同时丢线的行数
extern int Longest_White_Column_Left[2];    //最长白列: [0]长度(即 Search_Stop_Line), [1]所在列号
extern int Longest_White_Column_Right[2];   //最长白列: [0]长度(即 Search_Stop_Line), [1]所在列号
extern int Left_Lost_Flag[IMG_ROW] ;        //逐行左边界丢线标志, 丢线为1, 正常为0
extern int Right_Lost_Flag[IMG_ROW];        //逐行右边界丢线标志, 丢线为1, 正常为0
extern volatile int Zebra_Stripes_Flag;     //斑马线标志
/* ---- 丢图保护(冲出赛道自动停车) ---- */
extern volatile int Img_Lost_Cnt;           //判据连续成立的帧数, 0 = 正常
extern volatile int Img_Lost_Flag;          //1 = 这一轮跑动中因为丢图停过车
extern volatile int Img_Lost_On;            //1 = 开保护, 0 = 关(菜单4 可切换)
void Img_Lost_Tick(void);                   //每个摄像头帧调用一次
/* ---- 十字相关 ---- */
extern volatile int Left_Up_Find;           //四个拐点所在行号, 0 = 没找到
extern volatile int Right_Up_Find;
extern volatile int Left_Down_Find;
extern volatile int Right_Down_Find;
void Find_Up_Point(int start,int end);
void Find_Down_Point(int start,int end);
void Lengthen_Left_Boundry(int start,int end);
void Lengthen_Right_Boundry(int start,int end);
void Left_Add_Line(int x1,int y1,int x2,int y2);
void Right_Add_Line(int x1,int y1,int x2,int y2);
void Cross_Detect(void);
void Calculate_Mid_Line(void);
void TFT180_Show_Track_Lines(
    uint16 x,
    uint16 y,
    uint16 dis_width,
    uint16 dis_height
);
/* ---- 斑马线 ---- */
typedef enum
{
    ZEBRA_STATE_IDLE = 0,       /* 还没看到斑马线 */
    ZEBRA_STATE_CROSSING,       /* 已经看到, 正在通过(第1次或第2次) */
    ZEBRA_STATE_REARM,          /* 第1次已越过, 绕圈中, 等再次看到斑马线 */
    ZEBRA_STATE_PASSED,         /* 第2次已越过, 再走一小段就停 */
    ZEBRA_STATE_DONE            /* 已停车 */
}zebra_state_enum;

extern volatile uint8  Zebra_Task_State;    /* 当前状态, 菜单1可看 */
extern volatile uint8  Zebra_Seen_Flag;     /* 本帧是否看到斑马线 */
extern volatile uint8  Zebra_Pass_Cnt;      /* 已经越过斑马线几次 (0/1/2) */
extern volatile uint32 Zebra_Lost_Dist;     /* 看不到斑马线后累计走的距离, 用来标定 ZEBRA_CONFIRM_DIST */
extern volatile uint32 Zebra_Rearm_Dist;    /* 第1次越过后累计走的距离, 用来标定 ZEBRA_REARM_DIST */

uint8 Zebra_Detect(void);                   /* 单帧检测, 返回 1/0 */
void  Zebra_Task_Reset(void);               /* 发车时复位 */
void  Zebra_Task_Tick(void);                /* 每个摄像头帧调用一次 */
void  Zebra_Task_Distance(int speed_l, int speed_r);    /* TIM6 每 10ms 调用 */

/* ---- 直道标准赛宽标定(菜单5) ---- */
extern uint8    Base_Width[IMG_ROW];        /* 每行的标准赛宽(像素), 0 = 该行无效 */
extern uint8    Base_Width_Ok[IMG_ROW];     /* 该行是否标定成功 */
extern uint8    Base_Width_Range[IMG_ROW];  /* 该行的最大波动(像素), 越小越可信 */
extern uint8    Base_Width_Bad;             /* 无效行的数量 */
extern uint8    Base_Width_MaxRng;          /* 所有有效行里最大的波动值 */
extern volatile uint8 Width_Calib_State;    /* 0 空闲  1 采集中  2 采集完成 */
extern volatile uint8 Width_Calib_Prog;     /* 采集进度 */

extern const uint8 Base_Width_Default[IMG_ROW]; /* 固化的标准赛宽(开机初值), 在 img_process.c 里改 */

void Width_Calib_Start(void);               /* 开始一次标定 */
void Width_Calib_Clear(void);               /* 恢复成固化的初值 */
void Width_Calib_LoadDefault(void);         /* 把固化值装进 Base_Width[], 开机调一次 */
void Width_Calib_Tick(void);                /* 每个摄像头帧调用一次 */

#endif
