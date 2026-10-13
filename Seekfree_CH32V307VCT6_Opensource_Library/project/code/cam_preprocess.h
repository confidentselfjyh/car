/**
* @filename: cam_preprocess[摄像头图像预处理]
* @filepath: /MineSrc/app/CamAlgorithm
* @function: 摄像头图像的预处理，包括压缩、二值化等
* @author:   CyberLab-AI_Vision
*/

#ifndef CAM_PREPROCESS
#define CAM_PREPROCESS

#include "zf_common_headfile.h"
#include "Ourcode_headfile.h"

#define BASIC_LR_JUMP_BLACK(x,y)    (CDM_TEMP_POINT_BLACK(x,y)&&(CDM_TEMP_POINT_WHITE(x+1,y)||CDM_TEMP_POINT_WHITE(x-1,y))) //横向黑色跳变点，注意数组不可越界
#define BASIC_LR_JUMP_WHITE(x,y)    (CDM_TEMP_POINT_WHITE(x,y)&&(CDM_TEMP_POINT_BLACK(x+1,y)||CDM_TEMP_POINT_BLACK(x-1,y)))// 横向白色跳变点，注意数组不可越界
typedef struct
{
  int x;  //横坐标
  int y;  //纵坐标
}_POINT_; //左上角为起始点(0,0)

/* 图像大小定义
   MT9V03X 原图 188x120, 这里 94x60 正好是两个方向都精确 2:1 降采样,
   长宽比不失真(原来 60x60 横向是 3.13:1, 斜线斜率被放大 1.57 倍)
   注意: img_process.h 里有一份同样的定义, 改这里必须同步改那边 */
#define	IMG_ROW		60
#define	IMG_COL		94

/* 图像边界定义 */
#define IMG_BLACK       (0) // black point
#define IMG_WHITE       (1) // white point

#define ERROR_LINE      (0)   // line<=ERROR_LINE,the image is not true

#define IMG_WIDTH    (IMG_COL)                 //图像的宽
#define IMG_HEIGHT   (IMG_ROW)                  //图像的高
#define IMG_TOP         (5)   // 0
#define IMG_BOTTOM      (IMG_ROW-1)  // IMG_HEIGHT - 1
#define IMG_LEFT        (0)   // 0
#define IMG_RIGHT       (IMG_COL-1)  // IMG_WIDTH - 1
#define IMG_CENTER      (IMG_COL/2)  // (IMAGE_RIGHT - IMAGE_LEFT) >> 1
#define IMG_ERROR    (19)                   //远方误差(19)

#define IMG_BUTTOM   (IMG_BOTTOM)        //图像底部

/* 图像像素值定义 */
#define IMG_BLACK       (0) // black point
#define IMG_WHITE       (1) // white point


#define WT0(x,y) (BinaryImg_CDM[(y)][(x)] == IMG_WHITE) //白：x为列数，y为行数
#define BK0(x,y) (BinaryImg_CDM[(y)][(x)] == IMG_BLACK) //黑：x为列数，y为行数
//周围8点的黑白情况判断
#define WT7(x,y) (WT0(x-1,y))   //左7白
#define WT3(x,y) (WT0(x+1,y))   //右3白
#define WT5(x,y) (WT0(x,y-1))   //上5白
#define WT1(x,y) (WT0(x,y+1))   //下1白
#define WT6(x,y) (WT0(x-1,y-1)) //左上6白
#define WT8(x,y) (WT0(x-1,y+1)) //左下8白
#define WT4(x,y) (WT0(x+1,y-1)) //右上4白
#define WT2(x,y) (WT0(x+1,y+1)) //右下2白
#define BK7(x,y) (BK0(x-1,y))   //左7黑
#define BK3(x,y) (BK0(x+1,y))   //右3黑
#define BK5(x,y) (BK0(x,y-1))   //上5黑
#define BK1(x,y) (BK0(x,y+1))   //下1黑
#define BK6(x,y) (BK0(x-1,y-1)) //左上6黑
#define BK8(x,y) (BK0(x-1,y+1)) //左下8黑
#define BK4(x,y) (BK0(x+1,y-1)) //右上4黑
#define BK2(x,y) (BK0(x+1,y+1)) //右下2黑
//中心是白，邻点是黑
#define WT0_BK7(x,y) (WT0(x,y) && BK7(x,y)) // 左 7黑
#define WT0_BK3(x,y) (WT0(x,y) && BK3(x,y)) // 右 3黑
#define WT0_BK5(x,y) (WT0(x,y) && BK5(x,y)) // 上 5黑
#define WT0_BK1(x,y) (WT0(x,y) && BK1(x,y)) // 下 1黑
#define WT0_BK6(x,y) (WT0(x,y) && BK6(x,y)) //左上6黑
#define WT0_BK8(x,y) (WT0(x,y) && BK8(x,y)) //左下8黑
#define WT0_BK4(x,y) (WT0(x,y) && BK4(x,y)) //右上4黑
#define WT0_BK2(x,y) (WT0(x,y) && BK2(x,y)) //右下2黑
//中心是白，邻点是白
#define WT0_WT7(x,y) (WT0(x,y) && WT7(x,y)) // 左 7白
#define WT0_WT3(x,y) (WT0(x,y) && WT3(x,y)) // 右 3白
#define WT0_WT5(x,y) (WT0(x,y) && WT5(x,y)) // 上 5白
#define WT0_WT1(x,y) (WT0(x,y) && WT1(x,y)) // 下 1白
#define WT0_WT6(x,y) (WT0(x,y) && WT6(x,y)) //左上6白
#define WT0_WT8(x,y) (WT0(x,y) && WT8(x,y)) //左下8白
#define WT0_WT4(x,y) (WT0(x,y) && WT4(x,y)) //右上4白
#define WT0_WT2(x,y) (WT0(x,y) && WT2(x,y)) //右下2白
//中心是黑，邻点是白
#define BK0_WT7(x,y) (BK0(x,y) && WT7(x,y)) // 左 7白
#define BK0_WT3(x,y) (BK0(x,y) && WT3(x,y)) // 右 3白
#define BK0_WT5(x,y) (BK0(x,y) && WT5(x,y)) // 上 5白
#define BK0_WT1(x,y) (BK0(x,y) && WT1(x,y)) // 下 1白
#define BK0_WT6(x,y) (BK0(x,y) && WT6(x,y)) //左上6白
#define BK0_WT8(x,y) (BK0(x,y) && WT8(x,y)) //左下8白
#define BK0_WT4(x,y) (BK0(x,y) && WT4(x,y)) //右上4白
#define BK0_WT2(x,y) (BK0(x,y) && WT2(x,y)) //右下2白
//中心是黑，邻点是黑
#define BK0_BK7(x,y) (BK0(x,y) && BK7(x,y)) // 左 7黑
#define BK0_BK3(x,y) (BK0(x,y) && BK3(x,y)) // 右 3黑
#define BK0_BK5(x,y) (BK0(x,y) && BK5(x,y)) // 上 5黑
#define BK0_BK1(x,y) (BK0(x,y) && BK1(x,y)) // 下 1黑
#define BK0_BK6(x,y) (BK0(x,y) && BK6(x,y)) //左上6黑
#define BK0_BK8(x,y) (BK0(x,y) && BK8(x,y)) //左下8黑
#define BK0_BK4(x,y) (BK0(x,y) && BK4(x,y)) //右上4黑
#define BK0_BK2(x,y) (BK0(x,y) && BK2(x,y)) //右下2黑



//这个暂时先放在这里
#define		True		1
#define		False		0

/* 连通域相关定义 */
#define MAX_AREA    (255) // [连通域递归次数相关]注意：uint8_t 不可以大于 255
//连通域输出图像宏定义
#define CDM_TEMP_BINARY 						 BinaryImg_CDM
#define CDM_TEMP_POINT_WHITE(x,y)		(BinaryImg_CDM[y][x] == 1)
#define CDM_TEMP_POINT_BLACK(x,y)		(BinaryImg_CDM[y][x] == 0)
#define CDM_LAST_IMG_BLACK(x,y)                 (Last_IMG[y][x] == 0)
#define CDM_TEMP_POINT(x,y)		        (BinaryImg_CDM[y][x])
#define CDM_TEMP_POINT_SET(x,y)			(BinaryImg_CDM[y][x] = 1)
#define CDM_TEMP_POINT_RESET(x,y)		(BinaryImg_CDM[y][x] = 0)


//图像初步配置结构体
typedef struct
{
	int Threshold;       /* 阈值 */
	int minThres;				 /* 最小阈值 */
	int maxThres;				 /* 最大阈值 */
	float white_threshold;
}Img_Config;
extern Img_Config ImgConf;

//连通域相关结构体
typedef struct
{
	int16_t stRun[MAX_AREA];       // 区域起始点
	int16_t enRun[MAX_AREA];       // 区域终点
	int16_t rowRun[MAX_AREA];      // 区域所在行
	int16_t numOfRuns;             // 寻找到的总区域

	int16_t eq_l[MAX_AREA];        // 等价区域左标
	int16_t eq_r[MAX_AREA];        // 等价区域右标，即同一下标的 eq_l 和 eq_r 为连通区域标号
	int16_t peq;                   // 等价区域个数，最后一个等价区域为	peq - 1

	int16_t equival[MAX_AREA];     // 主要等价区域表，存储例：1，0，1，0，1，0，1，1，1，0，1，0，1...
	int16_t lastSign_equival;	   	 // 等价表中，最后一个等价区域的标号
}ConDomainNode;
extern ConDomainNode cdm;

extern volatile uint16 Img_White_Cnt;           //整幅二值图的白点总数(丢图保护用)

extern uint8 BinaryImg[IMG_ROW][IMG_COL];				//二值化图像
extern uint8 BinaryImg_CDM[IMG_ROW][IMG_COL];		//连通域后输出的二值化图像(只存 0/1)
extern uint8 CmpressedImg[IMG_ROW][IMG_COL];

void Connected_Domain(ConDomainNode *cdm, int16 find_back);
void CDM_Segment(ConDomainNode *cdm);
extern void CDM_deal(void);

int16 CDM_MainStreet_Segment_LEFT_UP(ConDomainNode *cdm, int16 LEFT, int16 TOP, _POINT_ *bot, _POINT_ *top);
int16 CDM_MainStreet_Segment_RIGHT_UP(ConDomainNode *cdm, int16 RIGHT, int16 TOP, _POINT_ *bot, _POINT_ *top);

/**
 * @description: 找主道图像分割点
 * @Author: Wei Zhou
 * @param {type} cdm    {ConDomainNode *} 保存连通域算法信息的结构体对象指针
 *               ele    {ELEMENT_SIGN} 元素类型
 */
// 搜索下一个点的方向
typedef enum
{
    MainStreet_LEFT,
    MainStreet_RIGHT
} MainStreetType;

extern int16 CDM_MainStreet_Point(ConDomainNode *cdm, MainStreetType mst, _POINT_ *point);


void JumpInit(void);
void ImgPreprocess(void);

int my_adapt_threshold(uint8 *image, uint16 col, uint16 row);

#endif

