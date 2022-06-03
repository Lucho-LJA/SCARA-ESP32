#ifndef _MOVE_OMNI_ROB_
    #define _MOVE_OMNI_ROB_
    #include "config.h"
	#include <Arduino.h>
    #include "ESP32_AnalogWrite.h"
    #include "ESP32_L298N.h"
    #include "ESP32_Encoder.h"
    //#include "ROS_CONFIG.h"

    L298N motors_43(PWM_BITS,MOTOR4_PWM,MOTOR4_IN_A,MOTOR4_IN_B,MOTOR3_IN_A,MOTOR3_IN_B,MOTOR3_PWM);
    L298N motors_21(PWM_BITS,MOTOR2_PWM,MOTOR2_IN_A,MOTOR2_IN_B,MOTOR1_IN_A,MOTOR1_IN_B,MOTOR1_PWM);

    Encoder EMotor_1(MOTOR1_ENCODER_A,MOTOR1_ENCODER_B,CHIHAI_E_PPR,WHEEL_DIAMETER);
    Encoder EMotor_2(MOTOR2_ENCODER_A,MOTOR2_ENCODER_B,CHIHAI_E_PPR,WHEEL_DIAMETER);
    Encoder EMotor_3(MOTOR3_ENCODER_A,MOTOR3_ENCODER_B,CHIHAI_E_PPR,WHEEL_DIAMETER);
    Encoder EMotor_4(MOTOR4_ENCODER_A,MOTOR4_ENCODER_B,CHIHAI_E_PPR,WHEEL_DIAMETER);
    
    void stopCar()
    {

        motors_21.stopMotor();
        motors_43.stopMotor();

    }
    void stopCarF()
    {

        motors_21.stopMotorF();
        motors_43.stopMotorF();

    }
    void moveCar()
    {
        motors_21.mbMotor(PWM_motor[1],PWM_motor[0]);
        motors_43.bmMotor(PWM_motor[3],PWM_motor[2]);

    }
    void backCar()
    {
        motors_21.bmMotor(PWM_motor[1],PWM_motor[0]);
        motors_43.mbMotor(PWM_motor[3],PWM_motor[2]);
    }
    void leftCar()
    {
        motors_21.moveMotor(PWM_motor[1],PWM_motor[0]);
        motors_43.moveMotor(PWM_motor[3],PWM_motor[2]);
    }
    void rightCar()
    {
        motors_21.backMotor(PWM_motor[1],PWM_motor[0]);
        motors_43.backMotor(PWM_motor[3],PWM_motor[2]);
    }
    void diagonalMR()
    {
        motors_21.backMotor34(PWM_motor[0]);
        motors_43.backMotor12(PWM_motor[3]);
    }
    void diagonalRB()
    {
        motors_21.backMotor12(PWM_motor[1]);
        motors_43.backMotor34(PWM_motor[2]);
    }
    void diagonalBL()
    {
        motors_21.moveMotor34(PWM_motor[0]);
        motors_43.moveMotor12(PWM_motor[3]);
    }
    void diagonalLM()
    {
        motors_21.moveMotor12(PWM_motor[1]);
        motors_43.moveMotor34(PWM_motor[2]);
    }
    void turnClockwise()
    {
        motors_21.backMotor(PWM_motor[1],PWM_motor[0]);
        motors_43.moveMotor(PWM_motor[3],PWM_motor[2]);

    }
    void turnCounterClockwise()
    {
        motors_21.moveMotor(PWM_motor[1],PWM_motor[0]);
        motors_43.backMotor(PWM_motor[3],PWM_motor[2]);
    }
#endif