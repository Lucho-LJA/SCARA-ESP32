#include "CONFIG.h"
#ifndef PID_H
    #define PID_H
    #include "Arduino.h"
    #include "config.h"

    float integral_[N_MOTOR]={0,0,0};
    float prev_integral_[N_MOTOR]={0,0,0};
    float derivative_[N_MOTOR]={0,0,0};
    float prev_error_[N_MOTOR]={0,0,0};
    float proporcional_[N_MOTOR]={0,0,0};
    float error[N_MOTOR]={0,0,0};
    int med_ang_[N_MOTOR]={0,0,0};

    #ifdef SCARA_POT_MOSTER
        void iterPID(int pwm_max_,int index_, int tol_){
            int pid_val=0;
            //set_point_ is constrained between min and max to prevent pid from having too much error
            error[index_] = setMotor[index_] - med_ang_[index_];
            if (error[index_] <= tol_){
                proporcional_[index_]=kp_motor[index_]*error[index_];
                integral_[index_] = prev_integral_[index_]+ki_motor[index_]*error[index_];//*dt_board/1000;
                derivative_[index_] = kd_motor[index_]*(error[index_] - prev_error_[index_]);//*1000/dt_board;
                pid_val = (int)(PWM_motor[index_]+proporcional_[index_]+integral_[index_]+derivative_[index_]);
                //Serial.println(pid_val);
                if (pid_val> pwm_max_)
                {
                    PWM_motor[index_]=pwm_max_;
                }else if(pid_val<-pwm_max_)
                {
                    PWM_motor[index_]=-pwm_max_;
                }else
                {
                    PWM_motor[index_]=pid_val;
                }
                prev_error_[index_]=error[index_];
                prev_integral_[index_]=integral_[index_];
                pid_val=0;
            }else{
                PWM_motor[index_]=0;
                error[index_]=0;
                prev_error_[index_]=error[index_];
                integral_[index_]=0;
                prev_integral_[index_]=integral_[index_];
                pid_val=0;
            }

        }

        void PIDcompute(int pwm_max_= PWM_MAX)
        {
            for (int i = 0; i < N_MOTOR-1; i++)
            {
                iterPID(pwm_max_,i, TOL_ERROR_ANG);
            }
            iterPID(pwm_max_,N_MOTOR-1, TOL_ERROR_LONG);
        }
    #endif

#endif