#ifndef _ROS_CONFIG_
    #define _ROS_CONFIG_
    #include "config.h"
    #include "Arduino.h"
    #include "ros.h"
    #include "std_msgs/String.h"
    #include "std_msgs/Int8.h"
    #include "std_msgs/Int16.h"
    #include "std_msgs/Float32MultiArray.h"
    #include "std_msgs/Int32MultiArray.h"
    #include "std_msgs/Char.h"
    #include <string>

    const char* ssid     = ROUTER_SSID;
    const char* password = ROUTER_PASWORD;
    //Set the IP's configuration
    IPAddress ip(IP_ESP32);
    IPAddress gateway(IP_GATEWAY);
    IPAddress subnet(IP_SUBNET);

    // Set the rosserial socket server IP address
    IPAddress server(ROS_SERVER);
    // Set the rosserial socket server port
    const uint16_t serverPort = ROS_SERVER_PORT;

    ros::NodeHandle nh;
    
    char movimiento='K';
    char tipo_func='0';
    int8_t opc=0;

    /* VARIABLES TO TOPICS */

    String opc_omni = "omni2";
    String pRPM=opc_omni+"/rpm";
    String pMPU=opc_omni+"/mpu";
    String pSET=opc_omni+"/setpoint";
    String pPKP=opc_omni+"/pid_kp";
    String pPKI=opc_omni+"/pid_ki";
    String pPKD=opc_omni+"/pid_kd";
    String pMOV=opc_omni+"/movimiento";
    String pENCO=opc_omni+"/encoder";




    void Lectura_SETPOINT( const std_msgs::Float32MultiArray& msg)
    {
        
        for(int i=0;i<4;i++)
        {
            SET_motor[i]=(int) (map(msg.data[i],0,ALL_MOTOR_RPM,0,PWM_MAX));
        }
        

    }
    void Lectura_KP_PID( const std_msgs::Float32MultiArray& msg)
    { 
        for(int i=0;i<4;i++)
        {
            kp_motor[i]=msg.data[i];
        }
    }

    void Lectura_KI_PID( const std_msgs::Float32MultiArray& msg)
    {
        
        for(int i=0;i<4;i++)
        {
            ki_motor[i]=msg.data[i];
        }
    }
    
    void Lectura_KD_PID( const std_msgs::Float32MultiArray& msg)
    {
        for(int i=0;i<4;i++)
        {
            kd_motor[i]=msg.data[i];
        }
    }
    
    void Lectura_mov( const std_msgs::Char& msg)
    {
        
        movimiento=msg.data;

    }

    
    // Make a chatter publisher
    std_msgs::Float32MultiArray rpm_msg;
    std_msgs::Float32MultiArray mpu_msg;
    std_msgs::Int32MultiArray encoder_msg;

    ros::Publisher omni_rpm(pRPM.c_str(), &rpm_msg);
    ros::Publisher omni_mpu(pMPU.c_str(), &mpu_msg);
    ros::Publisher omni_encoder(pENCO.c_str(), &encoder_msg);
    
    ros::Subscriber<std_msgs::Float32MultiArray> omni_setpoint(pSET.c_str(),&Lectura_SETPOINT);
    ros::Subscriber<std_msgs::Float32MultiArray> omni_kp(pPKP.c_str(),&Lectura_KP_PID);
    ros::Subscriber<std_msgs::Float32MultiArray> omni_ki(pPKI.c_str(),&Lectura_KI_PID);
    ros::Subscriber<std_msgs::Float32MultiArray> omni_kd(pPKD.c_str(),&Lectura_KD_PID);
    ros::Subscriber<std_msgs::Char> omni_mov(pMOV.c_str(),&Lectura_mov);


#endif