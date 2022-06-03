
//#define _INIT_MPU_ /**uncomment to calibrate MPU by first time**/
//#define_ALONE_ACT_
#include "config.h"
#ifndef _INIT_MPU_
  
  #include "ESP32_Encoder.h"
  #include "ROS_CONFIG.h"
  #include "MOVE_OMNI_ROB.h"
  #include <string>
  #include "PID_control.h"
  #ifndef _ALONE_ACT_
    #include "MPU6050_LECTURA.h"
  #endif
  String ip_board=" ";
  //Variables MPU



#else
  #include "INIT_MPU6050.h"
#endif





unsigned long prev_time_board=0;
unsigned long time_board=0;





void setup()
{

  #ifndef _INIT_SERIAL_
    #define _INIT_SERIAL_
    Serial.begin(115200);
  #endif
  #ifdef _INIT_MPU_
    mpu_calibration();
  #else
    //Iniciar MPU6050
    #ifndef _ALONE_ACT_
      Iniciar_MPU6050();
    #endif


    #include "Init_WIFI.h"

    
    #include "Init_ROS.h"
    
    stopCar();
    EMotor_1.reset();
    EMotor_2.reset();
    EMotor_3.reset();
    EMotor_4.reset();
  #endif
  prev_time_board =millis();
}


void loop()
{
  #ifndef _INIT_MPU_
    time_board = millis();
    if(time_board-prev_time_board>=dt_board)
    {
      prev_time_board=time_board;
      /*Serial.println("MEDICION RPM - PWM");
      Serial.print(med_rpm_[0]);
      Serial.print(" - ");
      Serial.print(med_rpm_[1]);
      Serial.print(" - ");
      Serial.print(med_rpm_[2]);
      Serial.print(" - ");
      Serial.println(med_rpm_[3]);
      
      Serial.println("SETPOINT");
      Serial.print(SET_motor[0]);
      Serial.print(" - ");
      Serial.print(SET_motor[1]);
      Serial.print(" - ");
      Serial.print(SET_motor[2]);
      Serial.print(" - ");
      Serial.println(SET_motor[3]);*/
      PIDcompute(PWM_MAX);
      /*PWM_motor[0]=SET_motor[0];
      PWM_motor[1]=SET_motor[1];
      PWM_motor[2]=SET_motor[2];
      PWM_motor[3]=SET_motor[3];*/
      /*Serial.println("PWM");
      Serial.print(PWM_motor[0]);
      Serial.print(" - ");
      Serial.print(PWM_motor[1]);
      Serial.print(" - ");
      Serial.print(PWM_motor[2]);
      Serial.print(" - ");
      Serial.println(PWM_motor[3]);*/
    }
      if (nh.connected()) 
        {
        
          omni_rpm.publish( &rpm_msg );
          omni_mpu.publish( &mpu_msg );
          omni_encoder.publish(&encoder_msg);
          // Say hello
          //Serial.println(movimiento);
          #include "omni_move_case.h"

        } else {
          Serial.println("Not Connected RASPBERRY");
          stopCar();
        }

        #ifndef _ALONE_ACT_
          Leer_mpu6050();
          mpu_msg.data= MPU_motor;
        #endif

        ENCODER_read[0]=EMotor_1.read();
        ENCODER_read[1]=EMotor_2.read();
        ENCODER_read[2]=EMotor_3.read();
        ENCODER_read[3]=EMotor_4.read();
        encoder_msg.data=ENCODER_read;

        RPM_motor[0]=abs(EMotor_1.getRPM());
        med_rpm_[0]=map(RPM_motor[0],0,MOTOR_1_RPM,0,PWM_MAX);
        RPM_motor[1]=abs(EMotor_2.getRPM());
        med_rpm_[1]=map(RPM_motor[1],0,MOTOR_1_RPM,0,PWM_MAX);
        RPM_motor[2]=abs(EMotor_3.getRPM());
        med_rpm_[2]=map(RPM_motor[2],0,MOTOR_1_RPM,0,PWM_MAX);
        RPM_motor[3]=abs(EMotor_4.getRPM());
        med_rpm_[3]=map(RPM_motor[3],0,MOTOR_1_RPM,0,PWM_MAX);
        rpm_msg.data=RPM_motor;
        //Serial.print(med_rpm_[0]);

        

        /*Serial.print(encoder_msg.data[0]);
        Serial.print(" - ");
        Serial.print(encoder_msg.data[1]);
        Serial.print(" - ");
        Serial.print(encoder_msg.data[2]);
        Serial.print(" - ");
        Serial.println(encoder_msg.data[3]);*/


        
        nh.spinOnce();
        delay(dt_board/10);
   #endif
}
