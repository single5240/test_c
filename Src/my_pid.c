#include "my_pid.h"
#include <string.h>
#define abs(x) ((x) > (0) ? (x) : (-(x)))

void f_Integral_Limit(pid_parameter_t *pid);
void f_Output_Limit(pid_parameter_t *pid);
void pid_clear(pid_parameter_t *pid);

void PidInit(pid_parameter_t *pid, fp32 kp, fp32 ki, fp32 kd, fp32 maxout,fp32 maxierror, fp32 dead)
{
    memset(pid, 0, sizeof(pid_parameter_t));
    pid->User_Fun = NULL;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = maxout;
    pid->max_Ierror = maxierror;
    pid->deadband = dead;
}

fp32 PidCalculate(pid_parameter_t *pid, fp32 SetValue, fp32 ActualValue)
{
    pid->SetValue = SetValue;
    pid->ActualValue = ActualValue;
    pid->error = pid->SetValue - pid->ActualValue;
    pid->Derror = pid->error - pid->LastError;
    if (abs(pid->error) > pid->deadband) //死区
    {
        pid->Pout = pid->error * pid->Kp;
        pid->Ierror += pid->error;

        // 微分先行
				pid->Dout = pid->Kd * pid->Derror;

        // 积分限幅
				f_Integral_Limit(pid);
        pid->Iout = pid->Ki * pid->Ierror;

        pid->out = pid->Pout + pid->Iout + pid->Dout;

        // 输出限幅
				f_Output_Limit(pid);
    }
    else
    {
        pid_clear(pid);
    }
    pid->LastActualValue = pid->ActualValue;
    pid->LastSetValue = pid->SetValue;
    pid->LastDerror = pid->Derror;
    pid->LastError = pid->error;

    return pid->out;
}

void f_Integral_Limit(pid_parameter_t *pid)
{
    if (pid->Ierror > pid->max_Ierror)
    {
        pid->Ierror = pid->max_Ierror;
    }
    if (pid->Ierror < -(pid->max_Ierror))
    {
        pid->Ierror = -(pid->max_Ierror);
    }
}

void f_Output_Limit(pid_parameter_t *pid)
{
    if (pid->out > pid->max_out)
    {
        pid->out = pid->max_out;
    }
    if (pid->out < -(pid->max_out))
    {
        pid->out = -(pid->max_out);
    }
}

void pid_clear(pid_parameter_t *pid)
{
    pid->error = pid->LastError = 0.0f;
    pid->Derror = pid->LastDerror = pid->LastLastDerror = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = pid->Ierror = 0.0f;
    pid->ActualValue = pid->SetValue = pid->LastActualValue = pid->LastSetValue = 0.0f;
}

