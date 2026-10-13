
#include "zf_common_headfile.h"
#include "Ourcode_headfile.h"

int Threshold = 0;
volatile float angle = 0.0;

extern volatile int GO;
extern volatile int keynum;
extern volatile float Err;
extern volatile int Speed_Left_Real;
extern volatile int Speed_Right_Real;
extern volatile int Speed_Left_Set;
extern volatile int Speed_Right_Set;
extern volatile int Target_speed;
int main (void) {
    clock_init (SYSTEM_CLOCK_120M);
    debug_init();

    // ≥ı ºªØ
    Init();

    while (1)
    {
        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            ImgPreprocess();
            Longest_White_Column();
            Cross_Detect();
            Island_Detect3();            
            Calculate_Mid_Line();
            Err = Err_Sum1();
            Zebra_Task_Tick();
            Width_Calib_Tick();
            angle = LocaPid(&servo_pid, Err, 0.0f);
            servo_ctrl((int16)angle);
            tft_show();
        }

    }
}


