#ifndef __FILTER_H__
#define __FILTER_H__
#include "struct_typedef.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#define mat arm_matrix_instance_f32 //�������ṹ�ľ���ṹ��   ( ���� �������������Լ�����(����) )
#define mat_init arm_mat_init_f32   //��������ʼ��
#define mat_add arm_mat_add_f32     //�������ӷ�
#define mat_sub arm_mat_sub_f32     //����������
#define mat_mult arm_mat_mult_f32   //�������˷�
#define mat_trans arm_mat_trans_f32 //�������ת��
#define mat_inv arm_mat_inverse_f32 //���������

typedef struct
{
    float raw_value;
    float filtered_value[4];
    mat xhat, xhatminus, z, A, H, AT, HT, Q, R, p, Pminus, k;
} kalman_filter_t;

typedef struct
{
    float raw_value;
    float filtered_value[4];
    float xhat_data[4], xhatminus_data[4], z_data[2], Pminus_data[16], K_data[8];
    float P_data[16];
    float AT_data[16], HT_data[8];
    float A_data[16];
    float H_data[8];
    float Q_data[16];
    float R_data[4];
} kalman_filter_init_t;

typedef struct
{
    float X_last; //��һʱ�̵����Ž��  X(k-|k-1)
    float X_mid;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
    float P_mid;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     // kalman����
    float A;      //ϵͳ����
    float B;
    float Q;
    float R;
    float H;
} extKalman_t;

void KalmanCreate(extKalman_t *p, float T_Q, float T_R);
float KalmanFilter(extKalman_t *p, float dat);

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
void kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);

extern kalman_filter_init_t visual_kalman_filter_init;
extern kalman_filter_t visual_kalman_filter;

//������ֵ�˲����������㣩
typedef __packed struct
{
    fp32 Input;        //��ǰȡ��ֵ
    int32_t count_num; //ȡ������
    fp32 Output;       //�˲����
    fp32 Sum;          //�ۼ��ܺ�
    fp32 FIFO[250];    //����
    int32_t sum_flag;  //�Ѿ���250����־
} sliding_mean_filter_type_t;

//һ�׵�ͨ�˲�����
typedef __packed struct
{
    fp32 input;      //��������
    fp32 last_input; //�ϴ�����
    fp32 out;        //�˲����������
    fp32 num;        //�˲�����
} first_order_filter_type_t;

/* ��ͨ�˲� */
extern float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input);
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num);

/* ƽ���˲� */
float sliding_mean_filter(sliding_mean_filter_type_t *mean_filter, fp32 Input, int32_t num); //��ֵ�����˲�
void sliding_mean_filter_init(sliding_mean_filter_type_t *mean_filter);                      //��ֵ�����˲���ʼ�����ɲ��ã�ֱ�Ӷ���ṹ��ʱ����ֵ��

#endif
