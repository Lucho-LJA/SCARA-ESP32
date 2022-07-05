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
    #ifdef SCARA_POT_MOSTER 
        // VARIABLES TO TOPICS
        String name_robot = ROBOT_NAME;
        String pPot=name_robot+"/get_position";
        String pSensor=name_robot+"/get_sensor";
        String pKp=name_robot+"/get_kp";
        String pKi=name_robot+"/get_ki";
        String pKd=name_robot+"/get_kd";
        String sAct=name_robot+"/set_actuator";
        String sPos=name_robot+"/set_position";
        String sKp=name_robot+"/set_kp";
        String sKi=name_robot+"/set_ki";
        String sKd=name_robot+"/set_kd";
        String sStop=name_robot+"/stop";

        //FUNCTIONS TO SYBSCRIPTORS
        void ReadSetPoint( const std_msgs::Float32MultiArray& msg){
            
            setMotor[0]=(int) (map(msg.data[0],ANGLE_MIN1,ANGLE_MAX1,0,PWM_MAX));
            setMotor[1]=(int) (map(msg.data[1],ANGLE_MIN2,ANGLE_MAX2,0,PWM_MAX));
            setMotor[2]=(int) (map(msg.data[2],LONG_MIN3,LONG_MAX3,0,PWM_MAX));
        }
        #ifdef PID_CONTROL
            void ReadKp( const std_msgs::Float32MultiArray& msg){
                for(int i=0;i< N_MOTOR;i++){
                    kp_motor[i]=(int) (msg.data[i]);
                }
            }
            void ReadKi( const std_msgs::Float32MultiArray& msg){
                for(int i=0;i< N_MOTOR;i++){
                    ki_motor[i]=(int) (msg.data[i]);
                }
            }
            void ReadKd( const std_msgs::Float32MultiArray& msg){
                for(int i=0;i< N_MOTOR;i++){
                    kd_motor[i]=(int) (msg.data[i]);
                }
            }
        #endif
        void ReadActuator( const std_msgs::Int8& msg){
            setAct = msg.data;
        }
        void ReadStop( const std_msgs::Int8& msg){
            EmStop = msg.data;
        }

        //MAKE VARIABLES TO PUBLISHERS
        std_msgs::Float32MultiArray pot_msg;
        std_msgs::Float32MultiArray sensor_msg;
        #ifdef PID_CONTROL
            std_msgs::Float32MultiArray kp_msg;
            std_msgs::Float32MultiArray ki_msg;
            std_msgs::Float32MultiArray kd_msg;
        #endif

        //BUILD PUBLISHER
        ros::Publisher pRobotPot(pPot.c_str(), &pot_msg);
        ros::Publisher pRobotSensor(pSensor.c_str(), &sensor_msg);
        #ifdef PID_CONTROL
            ros::Publisher pRobotKp(pKp.c_str(), &kp_msg);
            ros::Publisher pRobotKi(pKi.c_str(), &ki_msg);
            ros::Publisher pRobotKd(pKd.c_str(), &kd_msg);
        #endif


        //BUILD SUBSCRIPTORS
        ros::Subscriber<std_msgs::Float32MultiArray> sRobotPos(sPos.c_str(),&ReadSetPoint);
        ros::Subscriber<std_msgs::Int8> sRobotAct(sAct.c_str(),&ReadActuator);
        #ifdef PID_CONTROL
            ros::Subscriber<std_msgs::Float32MultiArray> sRobotKp(sKp.c_str(),&ReadKp);
            ros::Subscriber<std_msgs::Float32MultiArray> sRobotKi(sKp.c_str(),&ReadKi);
            ros::Subscriber<std_msgs::Float32MultiArray> sRobotKd(sKp.c_str(),&ReadKd);
        #endif
        ros::Subscriber<std_msgs::Int8> sRobotStop(sStop.c_str(),&ReadStop);

    #endif

#endif