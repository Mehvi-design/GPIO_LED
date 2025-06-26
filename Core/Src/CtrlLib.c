#include "CtrlLib.h"
void First_Order_Transfer_Func_Init(first_order_transfer_func_t *transfer_func, float A1, float Parameter_B[2]){
    transfer_func->A1 = A1;
    transfer_func->B0 = Parameter_B[0];
    transfer_func->B1 = Parameter_B[1];
    transfer_func->Output = 0.0f;
    transfer_func->Output_L1 = 0.0f;
    transfer_func->Input = 0.0f;
    transfer_func->Input_L1 = 0.0f;
}

void First_Order_Transfer_Func_Compute(first_order_transfer_func_t *transfer_func){
    transfer_func->Output = transfer_func->B0 * transfer_func->Input +
                            transfer_func->B1 * transfer_func->Input_L1 -
                            transfer_func->A1 * transfer_func->Output_L1;
    transfer_func->Input_L1 = transfer_func->Input;
    transfer_func->Output_L1 = transfer_func->Output;
}

float First_Order_Transfer_Func_Compute_with_Parameter(first_order_transfer_func_t *transfer_func, float Input){
    transfer_func->Input = Input;
    First_Order_Transfer_Func_Compute(transfer_func);
    return transfer_func->Output;
}

void First_Order_Transfer_Func_Clear(first_order_transfer_func_t *transfer_func){
    transfer_func->Output = 0.0f;
    transfer_func->Output_L1 = 0.0f;
    transfer_func->Input = 0.0f;
    transfer_func->Input_L1 = 0.0f;
}

void Second_Order_Transfer_Func_Init(second_order_transfer_func_t *transfer_func, float Parameter_A[2], float Parameter_B[3]){
    transfer_func->A1 = Parameter_A[0];
    transfer_func->A2 = Parameter_A[1];
    transfer_func->B0 = Parameter_B[0];
    transfer_func->B1 = Parameter_B[1];
    transfer_func->B2 = Parameter_B[2];
    transfer_func->Output = 0.0f;
    transfer_func->Output_L1 = 0.0f;
    transfer_func->Output_L2 = 0.0f;
    transfer_func->Input = 0.0f;
    transfer_func->Input_L1 = 0.0f;
    transfer_func->Input_L2 = 0.0f;
}

void Second_Order_Transfer_Func_Compute(second_order_transfer_func_t *transfer_func){
    transfer_func->Output = A_X_B(transfer_func->B0,transfer_func->Input) +
                            A_X_B(transfer_func->B1,transfer_func->Input_L1) +
                            A_X_B(transfer_func->B2,transfer_func->Input_L2) -
                            A_X_B(transfer_func->A1,transfer_func->Output_L1) -
                            A_X_B(transfer_func->A2,transfer_func->Output_L2);
	
    if(transfer_func->Output >= transfer_func->upper_limit){
    	transfer_func->Output = transfer_func->upper_limit;
    }
    if(transfer_func->Output <= transfer_func->lower_limit){
        transfer_func->Output = transfer_func->lower_limit;
    }
	
    transfer_func->Input_L2 = transfer_func->Input_L1;
    transfer_func->Input_L1 = transfer_func->Input;
    transfer_func->Output_L2 = transfer_func->Output_L1;
    transfer_func->Output_L1 = transfer_func->Output;

}

float Second_Order_Transfer_Func_Compute_with_Parameter(second_order_transfer_func_t *transfer_func, float Input){
    transfer_func->Input = Input;
    Second_Order_Transfer_Func_Compute(transfer_func);
    return transfer_func->Output;
}

void Second_Order_Transfer_Func_Clear(second_order_transfer_func_t *transfer_func){
    transfer_func->Output = 0.0f;
    transfer_func->Output_L1 = 0.0f;
    transfer_func->Output_L2 = 0.0f;
    transfer_func->Input = 0.0f;
    transfer_func->Input_L1 = 0.0f;
    transfer_func->Input_L2 = 0.0f;
}

void Third_Order_Transfer_Func_Init(third_order_transfer_func_t *transfer_func, float Parameter_A[3], float Parameter_B[4]){
    transfer_func->A1 = Parameter_A[0];
    transfer_func->A2 = Parameter_A[1];
    transfer_func->A3 = Parameter_A[2];
    transfer_func->B0 = Parameter_B[0];
    transfer_func->B1 = Parameter_B[1];
    transfer_func->B2 = Parameter_B[2];
    transfer_func->B3 = Parameter_B[3];
    transfer_func->Output = 0.0f;
    transfer_func->Output_L1 = 0.0f;
    transfer_func->Output_L2 = 0.0f;
    transfer_func->Output_L3 = 0.0f;
    transfer_func->Input = 0.0f;
    transfer_func->Input_L1 = 0.0f;
    transfer_func->Input_L2 = 0.0f;
    transfer_func->Input_L3 = 0.0f;
}

void Third_Order_Transfer_Func_Compute(third_order_transfer_func_t *transfer_func){
    transfer_func->Output = A_X_B(transfer_func->B0,transfer_func->Input) +
                            A_X_B(transfer_func->B1,transfer_func->Input_L1) +
                            A_X_B(transfer_func->B2,transfer_func->Input_L2) +
                            A_X_B(transfer_func->B3,transfer_func->Input_L3) -
                            A_X_B(transfer_func->A1,transfer_func->Output_L1) -
                            A_X_B(transfer_func->A2,transfer_func->Output_L2) -
                            A_X_B(transfer_func->A3,transfer_func->Output_L3);
	
	if(transfer_func->Output >= transfer_func->upper_limit){
    	transfer_func->Output = transfer_func->upper_limit;
    }
    if(transfer_func->Output <= transfer_func->lower_limit){
        transfer_func->Output = transfer_func->lower_limit;
    }
	
	
    transfer_func->Input_L3 = transfer_func->Input_L2;
    transfer_func->Input_L2 = transfer_func->Input_L1;
    transfer_func->Input_L1 = transfer_func->Input;
    transfer_func->Output_L3 = transfer_func->Output_L2;
    transfer_func->Output_L2 = transfer_func->Output_L1;
    transfer_func->Output_L1 = transfer_func->Output;

}

void Third_Order_Transfer_Func_Clear(third_order_transfer_func_t *transfer_func){
    transfer_func->Output = 0.0f;
    transfer_func->Output_L1 = 0.0f;
    transfer_func->Output_L2 = 0.0f;
    transfer_func->Output_L3 = 0.0f;
    transfer_func->Input = 0.0f;
    transfer_func->Input_L1 = 0.0f;
    transfer_func->Input_L2 = 0.0f;
    transfer_func->Input_L3 = 0.0f;
}

float Third_Order_Transfer_Func_Compute_with_Parameter(third_order_transfer_func_t *transfer_func, float Input){
    transfer_func->Input = Input;
    Third_Order_Transfer_Func_Compute(transfer_func);
    return transfer_func->Output;
}

void Integrator_Init(integrator_t *integrator, float Ts,float Gain){
    integrator->Ts = Ts;
    integrator->K = Gain;
    integrator->KxTs =  A_X_B(Ts, Gain);
    integrator->Input = 0.0f;
    integrator->Output = 0.0f;
    integrator->Output_L1 = 0.0f;
}

void Integrator_Compute(integrator_t *integrator){
//    integrator->Output = A_X_B(integrator->KxTs, integrator->Input) + integrator->Output_L1;
	integrator->Output += integrator->K * integrator->Ts * integrator->Input;

	if(integrator->Output >= integrator->upper_limit){
    	integrator->Output = integrator->upper_limit;
    }
    if(integrator->Output <= integrator->lower_limit){
        integrator->Output = integrator->lower_limit;
    }
	
}

float Integrator_Compute_with_Parameter(integrator_t *integrator, float Input){
    integrator->Input = Input;
    Integrator_Compute(integrator);
    return integrator->Output;
}

void Integrator_Clear(integrator_t *integrator){
    integrator->Input = 0.0f;
    integrator->Output = 0.0f;
    integrator->Output_L1 = 0.0f;
}

void PID_Controller_Init(pid_controller_t *pid_controller, float Ts, float Kp, float Ki, float Kd, float N){
    pid_controller->Ts = Ts;
    pid_controller->Kp = Kp;
    pid_controller->Ki = Ki;
    pid_controller->Kd = Kd;
    pid_controller->N = N;
    pid_controller->Input = 0.0f;
    pid_controller->Input_L1 = 0.0f;
    pid_controller->Input_L2 = 0.0f;
    pid_controller->Output = 0.0f;
    pid_controller->Output_L1 = 0.0f;
    pid_controller->Output_L2 = 0.0f;

    pid_controller->A1 = N * Ts - 2;
    pid_controller->A2 = 1 - N * Ts;
    pid_controller->B0 = Kd * N + Kp;
    pid_controller->B1 = Kp * N * Ts + Ki * Ts - 2 * Kd * N - 2 * Kp;
    pid_controller->B2 = Kd * N - Kp * N * Ts - Ki * Ts + Kp + Ki * N * Ts * Ts;
}

void PID_Controller_Update_Parameter(pid_controller_t *pid_controller, float Ts, float Kp, float Ki, float Kd, float N){
    pid_controller->Ts = Ts;
    pid_controller->Kp = Kp;
    pid_controller->Ki = Ki;
    pid_controller->Kd = Kd;
    pid_controller->N = N;

    pid_controller->A1 = N * Ts - 2;
    pid_controller->A2 = 1 - N * Ts;
    pid_controller->B0 = Kd * N + Kp;
    pid_controller->B1 = Kp * N * Ts + Ki * Ts - 2 * Kd * N - 2 * Kp;
    pid_controller->B2 = Kd * N - Kp * N * Ts - Ki * Ts + Kp + Ki * N * Ts * Ts;
}

void PID_Controller_Clear_Output(pid_controller_t *pid_controller){
    pid_controller->Input = 0.0f;
    pid_controller->Input_L1 = 0.0f;
    pid_controller->Input_L2 = 0.0f;
    pid_controller->Output = 0.0f;
    pid_controller->Output_L1 = 0.0f;
    pid_controller->Output_L2 = 0.0f;
}

void PID_Controller_Compute(pid_controller_t *pid_controller){
    pid_controller->Output = A_X_B(pid_controller->B0,pid_controller->Input) +
                            A_X_B(pid_controller->B1,pid_controller->Input_L1) +
                            A_X_B(pid_controller->B2,pid_controller->Input_L2) -
                            A_X_B(pid_controller->A1,pid_controller->Output_L1) -
                            A_X_B(pid_controller->A2,pid_controller->Output_L2);
	
	if(pid_controller->Output >= pid_controller->upper_limit){
    	pid_controller->Output = pid_controller->upper_limit;
    }
    if(pid_controller->Output <= pid_controller->lower_limit){
        pid_controller->Output = pid_controller->lower_limit;
    }
	
    pid_controller->Input_L2 = pid_controller->Input_L1;
    pid_controller->Input_L1 = pid_controller->Input;
    pid_controller->Output_L2 = pid_controller->Output_L1;
    pid_controller->Output_L1 = pid_controller->Output;
}

float PID_Controller_Compute_with_Parameter(pid_controller_t *pid_controller, float Input){
    pid_controller->Input = Input;
    PID_Controller_Compute(pid_controller);
    return pid_controller->Output;
}

void SOGI_Init(sogi_t *sogi, float K, float w, float Ts){
    Integrator_Init(&sogi->Integrator_Dz, Ts, 1.0f);
    Integrator_Init(&sogi->Integrator_Qz, Ts, 1.0f);
    sogi->K = K;
    sogi->w = w;
    sogi->Input = 0.0f;
}

void SOGI_Compute(sogi_t *sogi){
    sogi->Integrator_Dz.Input = A_X_B((A_X_B((sogi->Input - sogi->Integrator_Dz.Output) , sogi->K )- sogi->Integrator_Qz.Output),sogi->w);
    sogi->Integrator_Qz.Input = A_X_B(sogi->Integrator_Dz.Output,sogi->w);
    Integrator_Compute(&sogi->Integrator_Dz);
    Integrator_Compute(&sogi->Integrator_Qz);
}
