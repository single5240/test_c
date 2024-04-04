#ifndef _MY_PID_H
#define _MY_PID_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"

#define MOTORA_S_KP 10
#define MOTORA_S_KI 0.01
#define MOTORA_S_KD 0
#define MOTORA_S_MAXOUT 5000
#define MOTORA_S_MAXIERROR 200
#define MOTORA_S_DEADBAND 100

#define MOTORA_P_KP 10
#define MOTORA_P_KI 0.01
#define MOTORA_P_KD 0
#define MOTORA_P_MAXOUT 3000
#define MOTORA_P_MAXIERROR 200
#define MOTORA_P_DEADBAND 100

typedef __packed struct Pid_parameter_t // pid结构体变量
{
	fp32 Kp;
	fp32 Ki;
	fp32 Kd;

	fp32 SetValue;
	fp32 LastSetValue;
	fp32 LastActualValue;
	fp32 ActualValue;

	fp32 Ierror;
	fp32 Pout;
	fp32 Iout;
	fp32 Dout;
	fp32 out;

	fp32 Derror; //微分项
	fp32 LastDerror;
	fp32 LastLastDerror;
	fp32 error; //误差项
	fp32 LastError;

	fp32 max_out; //最大输出

	/* 积分限幅 */
	fp32 max_Ierror; //最大积分输出
	/* 误差死区 */
	fp32 deadband;

	void (*User_Fun)(struct Pid_parameter_t *);
} pid_parameter_t;
	 
fp32 PidCalculate(pid_parameter_t *pid, fp32 SetValue, fp32 ActualValue);
void PidInit(pid_parameter_t *pid, fp32 kp, fp32 ki, fp32 kd,  fp32 maxout,fp32 maxierror, fp32 dead);
#ifdef __cplusplus
}
#endif
#endif /*_MY_PID_H */
