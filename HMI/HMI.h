#include"string.h"
#include"main.h"
#include"arm_math.h"
#define HMI_UART USART2
//__weak void setPID(float32_t kp,float32_t ki,float32_t kd);
void ftc(char *s,float32_t f);
void send_PID(float32_t kp,float32_t ki,float32_t kd);
void rec_val(void);
void ftc_s(char *s, float32_t f);

