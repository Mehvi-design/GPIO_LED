#ifndef CTRLLIB_H_INCLUDED
#define CTRLLIB_H_INCLUDED

/*** Basic Math Define Start ***/
#define A_PLUS_B(A,B)   (A+B)
#define A_MINUS_B(A,B)  (A-B)
#define A_X_B(A,B)      (A*B)
#define A_DIVIDE_B(A,B) (A/B)
/*** Basic Math Define End ***/
typedef struct {
/*
            B0 * z + B1
    H(z) = -------------;
               z + A1

    Output = B0 * Input + B1 * Input_L1 - A1 * Output_L1;
*/
    float Output;
    float Input;

    float Output_L1;
    float Input_L1;

    float A1;
    float B0;
    float B1;
	
	float upper_limit;
	float lower_limit;
} first_order_transfer_func_t;
void First_Order_Transfer_Func_Init(first_order_transfer_func_t *transfer_func, float A1, float Parameter_B[2]);
void First_Order_Transfer_Func_Compute(first_order_transfer_func_t *transfer_func);
float First_Order_Transfer_Func_Compute_with_Parameter(first_order_transfer_func_t *transfer_func, float Input);
void First_Order_Transfer_Func_Clear(first_order_transfer_func_t *transfer_func);

typedef struct{
/*
            B0 * z^2 + B1 * z + B2
    H(z) = ------------------------;
               z^2 + A1 * z + A2

    Output = B0 * Input + B1 * Input_L1 + B2 * Input_L2 - A1 * Output_L1 - A2 * Output_L2;
*/
    float Output;
    float Input;

    float Output_L1;
    float Output_L2;
    float Input_L1;
    float Input_L2;

    float A1;
    float A2;
    float B0;
    float B1;
    float B2;
	
	float upper_limit;
	float lower_limit;
} second_order_transfer_func_t;
void Second_Order_Transfer_Func_Init(second_order_transfer_func_t *transfer_func, float Parameter_A[2], float Parameter_B[3]);
void Second_Order_Transfer_Func_Compute(second_order_transfer_func_t *transfer_func);
float Second_Order_Transfer_Func_Compute_with_Parameter(second_order_transfer_func_t *transfer_func, float Input);
void Second_Order_Transfer_Func_Clear(second_order_transfer_func_t *transfer_func);

typedef struct{
/*
            B0 * z^3 + B1 * z^2 + B2 * z + B3
    H(z) = -----------------------------------;
               z^3 + A1 * z^2 + A2 * z + A2

    Output = B0 * Input + B1 * Input_L1 + B2 * Input_L2 + B3 * Input_L3 - A1 * Output_L1 - A2 * Output_L2 - A3 * Output_L3;
*/
    float Output;
    float Input;

    float Output_L1;
    float Output_L2;
    float Output_L3;
    float Input_L1;
    float Input_L2;
    float Input_L3;

    float A1;
    float A2;
    float A3;
    float B0;
    float B1;
    float B2;
    float B3;
	
	float upper_limit;
	float lower_limit;
} third_order_transfer_func_t;
void Third_Order_Transfer_Func_Init(third_order_transfer_func_t *transfer_func, float Parameter_A[3], float Parameter_B[4]);
void Third_Order_Transfer_Func_Compute(third_order_transfer_func_t *transfer_func);
float Third_Order_Transfer_Func_Compute_with_Parameter(third_order_transfer_func_t *transfer_func, float Input);
void Third_Order_Transfer_Func_Clear(third_order_transfer_func_t *transfer_func);

typedef struct {
/*
            K * Ts
    H(z) = --------;
            z - 1

    Output = K * Ts * Input + Output_L1 ;
*/

    float Ts;
    float Output;
    float Input;
    float K;

    float KxTs;
    float Output_L1;
	
	float upper_limit;
	float lower_limit;
} integrator_t;
void Integrator_Init(integrator_t *integrator, float Ts,float Gain);
void Integrator_Compute(integrator_t *integrator);
float Integrator_Compute_with_Parameter(integrator_t *integrator, float Input);
void Integrator_Clear(integrator_t *integrator);


typedef struct {
/*
                             1                     N
    H(z) = Kp + Ki * Ts * ------- + Kd * ---------------------;
                           z - 1          1 + N * Ts / (z - 1)

           (Kd * N + Kp) * z^2 +(Kp * N * Ts + Ki * Ts - 2 * Kd * N - 2 * Kp) * z + (Kd * N - Kp * N * Ts - Ki * Ts + Kp + Ki * N * Ts^2)
    H(z) = -------------------------------------------------------------------------------------------------------------------------------;
                                                  z^2 + (N * Ts - 2) * z + (1 - N * Ts)
*/
    float Ts;
    float Kp;
    float Ki;
    float Kd;
    float N;
    float Output;
    float Input;

    float A1;
    float A2;
    float B0;
    float B1;
    float B2;

    float Input_L1;
    float Input_L2;
    float Output_L1;
    float Output_L2;
	
	float upper_limit;
	float lower_limit;
} pid_controller_t;
void PID_Controller_Init(pid_controller_t *pid_controller, float Ts, float Kp, float Ki, float Kd, float N);
void PID_Controller_Update_Parameter(pid_controller_t *pid_controller, float Ts, float Kp, float Ki, float Kd, float N);
void PID_Controller_Clear_Output(pid_controller_t *pid_controller);
void PID_Controller_Compute(pid_controller_t *pid_controller);
float PID_Controller_Compute_with_Parameter(pid_controller_t *pid_controller, float Input);


typedef struct {
/*
              -------------------------------------------------------------------------
             |                                                                         |
             |    ----                                                                 |
              -->|-   |         ---        ----         ---        -----------------   |
     Input------>|+   |------->| K |----->|+   |------>| w |----->| K * Ts / (z - 1)|----->Output_Dz
                  ----          ---    -->|-   |        ---        -----------------   |
                                      |    ----                                        |
                                      |            ------------------------------------
                                      |           |
                                      |           |     ---        -----------------
                                      |            --->| w |----->| K * Ts / (z - 1)|----->Output_Qz
                                      |                 ---        -----------------   |
                                      |                                                |
                                       ------------------------------------------------

*/
    float Input;
    float K;
    float w;
    integrator_t Integrator_Dz;
    integrator_t Integrator_Qz;
} sogi_t;
void SOGI_Init(sogi_t *sogi, float K, float w, float Ts);
void SOGI_Compute(sogi_t *sogi);

typedef struct {
    float Input;
    float Output;
} single_phase_sogi_pll;

typedef struct{
    float Input_A;
    float Input_B;
    float Input_C;
    float Output;
} three_phase_sogi_pll;
#endif // CTRLLIB_H_INCLUDED
