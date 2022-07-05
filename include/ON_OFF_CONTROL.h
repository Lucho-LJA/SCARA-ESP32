#ifndef PID_H
    #define PID_H
    #include "Arduino.h"
    #include "CONFIG.h"
    #ifdef SCARA_POT_MOSTER 
        class ControlScara {
            private:
                int tol_ang=0;
                int tol_dist=0;
                int n_motor=0;
            public:
                ControlScara(int tol_ang_,int tol_dist_, int n_motor_);
                uint16_t *compute(int setM[], int medA[], uint16_t pwm[]);
            private:
                int iter(float error, int tol_);
                
        };
        
        ControlScara::ControlScara(int tol_ang_, int tol_dist_, int n_motor_){
            tol_ang = tol_ang_;
            tol_dist = tol_dist_;
            n_motor = n_motor_;
        }

        int ControlScara::iter(float error, int tol_){
            if (abs(error) >= tol_){
                if (error>0){
                    return 1;
                }else{
                    return -1;
                }
            }else{
                return 0;
            }
        }

        uint16_t *ControlScara::compute(int setM[], int medA[], uint16_t pwm[]){
            for (int i = 0; i < n_motor-1; i++)
                {
                    pwm[i]=iter((setM[i] - medA[i]), tol_ang);
                }
            pwm[n_motor-1]=iter((setM[n_motor-1] - medA[n_motor-1]), tol_dist);
            return pwm;
        }

    #endif

#endif