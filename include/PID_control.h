#ifndef PID_H
    #define PID_H
    #include "Arduino.h"
    #include "config.h"

    float integral_[4]={0,0,0,0};
    float prev_integral_[4]={0,0,0,0};
    float derivative_[4]={0,0,0,0};
    float prev_error_[4]={0,0,0,0};
    int med_rpm_[4]={0,0,0,0};

    void PIDcompute(int pwm_max_)
    {
        float proporcional_[4]={0,0,0,0};
        float error[4]={0,0,0,0};
        int pid_val=0;

        //set_point_ is constrained between min and max to prevent pid from having too much error
        for(int i=0;i<4;i++)
        {
            error[i] = SET_motor[i] - med_rpm_[i];
            proporcional_[i]=kp_motor[i]*error[i];
            integral_[i] = prev_integral_[i]+ki_motor[i]*error[i];//*dt_board/1000;
            derivative_[i] = kd_motor[i]*(error[i] - prev_error_[i]);//*1000/dt_board;
            pid_val = (int)(PWM_motor[i]+proporcional_[i]+integral_[i]+derivative_[i]);
            //Serial.println(pid_val);
            if (pid_val> pwm_max_)
            {
                PWM_motor[i]=pwm_max_;
            }else if(pid_val<0)
            {
                PWM_motor[i]=0;
            }else
            {
                PWM_motor[i]=pid_val;
            }
            prev_error_[i]=error[i];
            prev_integral_[i]=integral_[i];
            pid_val=0;
        }
    }

#endif
