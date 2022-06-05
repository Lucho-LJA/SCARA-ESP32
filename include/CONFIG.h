#ifndef _CONFIG_H_
    #define _CONFIG_H_
    #include <Arduino.h>
    #include <string>
    
    //================DEFAULT CONFIGURATION TO SCARA ROBOT=============================
    /*
        ROS CONFIGURATION - COMUNICATION
        ROBOT_NAME : name of robot to node (It must unique)
        ROUTER_SSID : SSID of router (Iqual as you see when scan a WiFi)
        ROUTER_PASSWORD : PASWORD in plain text
        IP_ESP32 : IP static (you should config a MAC in your Router)
        IP_GATEWAY : GETWAY of ROUTER
        IP_SUBNET : SUBNET (The same of router configuration)
        ROS_SERVER  : IP of machine where run roscore (server)
        ROS_SERVER_PORT : PORT where run ROS_SERVER (It is configured at ROS_SERVER package)
    */
    #define ROBOT_NAME "scara1"
    #define ROUTER_SSID "INTERNET ALLAUCA"
    #define ROUTER_PASWORD "2903NUis235689"
    #define IP_ESP32 192,168,1,152    //Use <,> and not <.> 
    #define IP_GATEWAY 192,168,1,1    //Use <,> and not <.> 
    #define IP_SUBNET 255,255,255,0   //Use <,> and not <.> 
    #define ROS_SERVER 192,168,1,112   //Use <,> and not <.> 
    #define ROS_SERVER_PORT 11422


    /*
        SELECT ROBOT-TYPE
        Uncomment the type of robot following the kind of motor
        SCARA_POT_MONSTER : Use motor with pot control without encoder and use MONSTER DRIVER
                            2 motor and 1 linear motor with pot
                            simple actuador ON/OFF
                            Use FUNCTION control or PID
                            
    */
    #define SCARA_POT_MOSTER


    /*
        SELECT BOARD DEVELOPMENT
        Uncomment the type of board
        ESP32_38P : ESP32 with 38 pins and standar configuration OF 10 bits
    */
    #define ESP32_38P
    /*
        Define the default velocity PWM-10BITS (0-1024)
        Define the defult POSITION ANGLE
        Define max and min angle
        Define pins of board
    */
    
    #ifdef SCARA_POT_MOSTER

        #define DEFAULT_POS1 0
        #define DEFAULT_POS2 0
        #define DEFAULT_POS3 0

        #define ANGLE_MIN1 0
        #define ANGLE_MAX1 180
        #define ANGLE_MIN2 0
        #define ANGLE_MAX2 180
        #define LONG_MIN3 0
        #define LONG_MAX3 180

        #define POT_MIN1 0
        #define POT_MAX1 5
        #define POT_MIN2 0
        #define POT_MAX2 5
        #define POT_MIN3 0
        #define POT_MAX3 5

        //pins of board
        #ifdef ESP32_38P
            //MOTOR_1
            #define MOTOR1_PWM 13
            //#define CHANEL_S1 15   
            #define MOTOR1_IN_A 14  
            #define MOTOR1_IN_B 12 
            #define MOTOR1_AMP 10
            #define MOTOR1_POT 10
            //MOTOR_2
            #define MOTOR2_PWM 25
            //#define CHANEL_S2 14   
            #define MOTOR2_IN_A 26  
            #define MOTOR2_IN_B 27
            #define MOTOR2_AMP 10
            #define MOTOR2_POT 10 
            //MOTOR_3
            #define MOTOR3_PWM 17
            //#define CHANEL_S3 13   
            #define MOTOR3_IN_A 4  
            #define MOTOR3_IN_B 16
            #define MOTOR3_AMP 10
            #define MOTOR3_POT 10
            //ACTUATOR
            #define PIN_ACTUATOR 10
            
        #endif
    #endif
    
    
    /*
        INPUT YOUR CONTROL PARAMETRES
        Uncomment the type of control that you want and configure it
    */
    #define PID_CONTROL //Config PID

    //#define PID_CONTROL
    #ifdef PID_CONTROL
        #define TOL_ERROR_ANG 20
        #define TOL_ERROR_LONG 100
        #define MOTOR1_KP 1
        #define MOTOR1_KI 0
        #define MOTOR1_KD 0
        #define MOTOR2_KP 1
        #define MOTOR2_KI 0
        #define MOTOR2_KD 0
        #define MOTOR3_KP 1
        #define MOTOR3_KI 0
        #define MOTOR3_KD 0
         
        
    #endif
    


    //================CONFIGURATION FOLLOWING YOUR SELECTION=============================
    // EMERGENCI STOP
    int EmStop = 0;
    //BOARDS
    #ifdef ESP32_38P
        // PWM Resolution of the microcontroller
        #define PWM_BITS 10                
        int PWM_MAX=pow(2, PWM_BITS) - 1;
        #define PWM_MIN 0
       // #define CORE_NUM_INTERRUPT 32
    #endif
//Correct the pins
    //MOTOR PINS
    #ifdef SCARA_POT_MOSTER
        //Variables to control
        #define N_MOTOR 3 //number of motors
        int setMotor[N_MOTOR]={DEFAULT_POS1,DEFAULT_POS2,DEFAULT_POS3}; //setpoint of motors position
        int setAct = 0; //activation of Actuator
        float potMotor[N_MOTOR]={0,0,0};
        float sensorMotor[N_MOTOR]={0,0,0};
        #define SAMPLE_ANG 1 //Sample min ang

    #endif 


    
    #ifdef PID_CONTROL
        #ifdef SCARA_POT_MOSTER
        uint16_t PWM_motor[N_MOTOR]={0,0,0};
        float kp_motor[N_MOTOR]={MOTOR1_KP,MOTOR2_KP,MOTOR3_KP};
        float ki_motor[N_MOTOR]={MOTOR1_KI,MOTOR2_KI,MOTOR3_KI};
        float kd_motor[N_MOTOR]={MOTOR1_KD,MOTOR2_KD,MOTOR3_KD};
        #define DT_BOARD 50 // delay system board in ms
        #endif
    #endif

#endif