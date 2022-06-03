#ifndef _CONFIG_H_
    #define _CONFIG_H_
    #include <Arduino.h>
    #include <string>
    
        //================DEFAULT CONFIGURATION IMU SENSOR=============================
    //uncomment the IMU you're using
    
    #define IMU_MPU6050
    #define MOTOR1_KP 1//21.4
    #define MOTOR1_KI 0//228
    #define MOTOR1_KD 0//0
    #define MOTOR2_KP 1//20.7
    #define MOTOR2_KI 0//276
    #define MOTOR2_KD 0//0
    #define MOTOR3_KP 1//20.8
    #define MOTOR3_KI 0//221
    #define MOTOR3_KD 0//0
    #define MOTOR4_KP 1//20.5
    #define MOTOR4_KI 0//183
    #define MOTOR4_KD 0//0
    
    
    int dt_board = 50; // delay system board in ms
    uint16_t PWM_motor[4]={0,0,0,0};
    float RPM_motor[4]={0,0,0,0};
    int SET_motor[4]={0,0,0,0};
    float MPU_motor[6]={0,0,0,0,0,0};
    int ENCODER_read[4]={0,0,0,0};

    float kp_motor[4]={MOTOR1_KP,MOTOR2_KP,MOTOR3_KP,MOTOR4_KP};
    float ki_motor[4]={MOTOR1_KI,MOTOR2_KI,MOTOR3_KI,MOTOR4_KI};
    float kd_motor[4]={MOTOR1_KD,MOTOR2_KD,MOTOR3_KD,MOTOR4_KD};

    float setpoint_motor[4]={0,0,0,0};

      //Variables de estado
    String Error_sistema =" ";
    
    //MPU6050 CONFIGURATION
    #define ACCEL_SCALE 1/8192
    #define G_TO_ACCEL 9.81
    //define connection of net
    #define OMNI_N "omni2"
    #define ROUTER_SSID "INTERNET ALLAUCA"
    #define ROUTER_PASWORD "2903LUis235689"
    #define IP_ESP32 192,168,1,152    //Use <,> and not <.> 
    #define IP_GATEWAY 192,168,1,1    //Use <,> and not <.> 
    #define IP_SUBNET 255,255,255,0   //Use <,> and not <.> 
    #define ROS_SERVER 192,168,1,112   //Use <,> and not <.> 
    #define ROS_SERVER_PORT 11422
    
    
    
    //uncomment the type of robot
    #define OMNI_ROB_V1

    //uncomment the type of develop board
    #define ESP32_38P
    #ifdef ESP32_38P
        //#define ESP32 //Definicion de librerias
    #endif

    //uncomment the base you're building
    //#define NORMAL        // round wheel drive robot
    #define MECANUM         // Mecanum drive robot

    //uncomment the motor driver you're using
    #define L298N_2_MOTOR
    #define L298N_2_DRIVERS

    //================DEFAULT CONFIGURATION TYPE OF MOTOR=============================
    /*
    ROBOT ORIENTATION OMNI-ROBO
            FRONT (SWITCH)
        MOTOR1  MOTOR2  
        MOTOR3  MOTOR4  (4WD/MECANUM/LRRL)  
            BACK
    */

    //uncomment the motor to wheel and put it on the motors (MAXIMUN RPM,PPR)
    #define CHIHAI_E_RPM_MAX 92     //micro gear with encoder chihai E 1:86 92RPM NO LOAD
    #define CHIHAI_E_RPM_MIN 70
    #define CHIHAI_E_PPR 7          //micro gear with encoder chihai E 1:86 92RPM NO LOAD
    #define MICRO_GEAR_100_ERPM_MAX 71//micro gear  with encode 1:100
    #define MICRO_GEAR_100_PPR 3//micro gear  with encoder 1:10

    #define REDUCTION 86 // 1:x--->x=86

    //select the motor or comment if NOT all motors use a especific motor
    #define ALL_MOTOR
    #define ALL_MOTOR_RPM CHIHAI_E_RPM_MAX
    #define ALL_MOTOR_PPR CHIHAI_E_PPR

    //specify on each motor and don't do it if ALL_MOTOR is uncomment
    #ifndef ALL_MOTOR
        #define MOTOR_1_RPM CHIHAI_E_RPM_MAX           // motor1's maximum RPM AND PPR
        #define MOTOR_2_RPM CHIHAI_E_RPM_MAX            // motor2's maximum RPM AND PPR
        #define MOTOR_3_RPM CHIHAI_E_RPM_MAX            // motor3's maximum RPM AND PPR
        #define MOTOR_4_RPM CHIHAI_E_RPM_MAX           // motor4's maximum RPM AND PPR
        #define MOTOR_1_PPR CHIHAI_E_PPR           // motor1's maximum PPR
        #define MOTOR_2_PPR CHIHAI_E_PPR          // motor2's maximum PPR
        #define MOTOR_3_PPR CHIHAI_E_PPR           // motor3's maximum PR
        #define MOTOR_4_PPR CHIHAI_E_PPR          // motor4's maximum PPR
    #else
        // motors's maximum RPM AND PPR
        #define MOTOR_1_RPM ALL_MOTOR_RPM            // motor1's maximum RPM AND PPR
        #define MOTOR_2_RPM ALL_MOTOR_RPM            // motor2's maximum RPM AND PPR
        #define MOTOR_3_RPM ALL_MOTOR_RPM            // motor3's maximum RPM AND PPR
        #define MOTOR_4_RPM ALL_MOTOR_RPM            // motor4's maximum RPM AND PPR
        #define MOTOR_1_PPR ALL_MOTOR_PPR            // motor1's maximum PPR
        #define MOTOR_2_PPR ALL_MOTOR_PPR           // motor2's maximum PPR
        #define MOTOR_3_PPR ALL_MOTOR_PPR            // motor3's maximum PR
        #define MOTOR_4_PPR ALL_MOTOR_PPR           // motor4's maximum PPR           
    #endif
    //================END DEFAULT CONFIGURATION TYPE OF MOTOR======================





    //================END DEFAULT CONFIGURATION IMU SENSOR=========================

    #define DEBUG 1


    //define your robot' specs here

    // wheel's specs
    #ifdef MECANUM
        #define WHEEL_DIAMETER 0.066        // wheel's diameter in meters
        #define LR_WHEELS_DISTANCE 0.235    // distance between left and right wheels
        #define FR_WHEELS_DISTANCE 0.30     // distance between front and rear wheels
    #endif
    //================END DEFAULT CONFIGURATION ROBOT PARAMETRES===================



    //================DEFAULT CONFIGURATION DEVELOP BOARD==========================
    #ifdef ESP32_38P
        #define PWM_BITS 10                // PWM Resolution of the microcontroller
        int PWM_MAX=pow(2, PWM_BITS) - 1;
        #define PWM_MIN 0
       // #define CORE_NUM_INTERRUPT 32
    #endif
    //================END DEFAULT CONFIGURATION DEVELOP BOARD=======================
    /*
    ROBOT ORIENTATION OMNI-ROBO
            FRONT (SWITCH)
        MOTOR1  MOTOR2  
        MOTOR3  MOTOR4  (4WD/MECANUM/LRRL)  
            BACK
    */

    //================DEFAULT CONFIGURATION ENCODER PIN=============================
    /// ENCODER PINS 
    #define MOTOR1_ENCODER_A 39  //({39,19})
    #define MOTOR1_ENCODER_B 19

    #define MOTOR2_ENCODER_A 18  //({18,5})
    #define MOTOR2_ENCODER_B 5

    #define MOTOR3_ENCODER_A 34 
    #define MOTOR3_ENCODER_B 35

    #define MOTOR4_ENCODER_A 36  //({36,23})
    #define MOTOR4_ENCODER_B 23


    //================END DEFAULT CONFIGURATION ENCODER PIN=========================


    //================DEFAULT CONFIGURATION MOTOR PIN===============================
    //MOTOR PINS
    #ifdef L298N_2_MOTOR

        //MOTOR_1
        #define MOTOR1_PWM 13   //13
        #define MOTOR1_IN_A 14  //14
        #define MOTOR1_IN_B 12  //12
        //MOTOR_2
        #define MOTOR2_PWM 25    //25
        #define MOTOR2_IN_A 26   //26
        #define MOTOR2_IN_B 27  //27
        //MOTOR_3
        #define MOTOR3_PWM 17   //17
        #define MOTOR3_IN_A 4  //4
        #define MOTOR3_IN_B 16  //16
        //MOTOR_4
        #define MOTOR4_PWM 33   //33
        #define MOTOR4_IN_A 32  //32
        #define MOTOR4_IN_B 15   //15

    #endif 



#endif