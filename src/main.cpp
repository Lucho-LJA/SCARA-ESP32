#include "CONFIG.h"
#include "ROS_CONFIG.h"
#include <string>

#ifdef PID_CONTROL
  #include "PID_CONTROL.h"
#endif

#ifdef ON_OFF_CONTROL
  #include "ON_OFF_CONTROL.h"
#endif

#ifdef ESP32_38P
  #include <ESP32_AnalogWrite.h>
  #include <ESP32AnalogRead.h>
  //Config readPIN
  ESP32AnalogRead sensor1;
  ESP32AnalogRead sensor2;
  ESP32AnalogRead sensor3;
  ESP32AnalogRead pot1;
  ESP32AnalogRead pot2;
  ESP32AnalogRead pot3;
#endif

#ifdef ON_OFF_CONTROL
  ControlScara scara(TOL_ERROR_ANG,TOL_ERROR_LONG,N_MOTOR);
#endif

String ip_board=" ";

unsigned long prev_time_board=0;
unsigned long time_board=0;

void setup()
{

  #ifndef _INIT_SERIAL_
    #define _INIT_SERIAL_
    #ifdef ESP32_38P
      Serial.begin(115200);
    #endif
  #endif
  #ifdef ESP32_38P
    WiFi.mode(WIFI_STA);
    WiFi.config(ip,gateway,subnet);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
      Serial.println("Connecting...");
      delay(500);
    }
    ip_board=String(WiFi.localIP());
    Serial.println(WiFi.localIP()); 
    
    // Set the connection to rosserial socket server
    nh.getHardware()->setConnection(server, serverPort);
        
    nh.initNode();
    //length to variables of publisher
    pot_msg.data_length=3;
    sensor_msg.data_length=3;

    #ifdef PID_CONTROL
      kp_msg.data_length=3;
      ki_msg.data_length=3;
      kd_msg.data_length=3;
    #endif

    // Start publish
    nh.advertise(pRobotPot);
    nh.advertise(pRobotSensor);
    #ifdef PID_CONTROL
      nh.advertise(pRobotKp);
      nh.advertise(pRobotKi);
      nh.advertise(pRobotKd);
    #endif
    //Start subscriptors
    nh.subscribe(sRobotPos);
    nh.subscribe(sRobotAct);
    nh.subscribe(sRobotStop);
    #ifdef PID_CONTROL
      nh.subscribe(sRobotKp);
      nh.subscribe(sRobotKi);
      nh.subscribe(sRobotKd);
    #endif
    

    //Config readPIN
    sensor1.attach(MOTOR1_AMP);
    sensor2.attach(MOTOR2_AMP);
    sensor3.attach(MOTOR3_AMP);
    pot1.attach(MOTOR1_POT);
    pot2.attach(MOTOR2_POT);
    pot3.attach(MOTOR3_POT);
    
    //Config pwmPIN
    analogWriteChannel(MOTOR1_PWM);
    analogWriteChannel(MOTOR2_PWM);
    analogWriteChannel(MOTOR3_PWM);
    analogWriteResolution(PWM_BITS); //Resolution of pins
    //set motor without move
    analogWrite(MOTOR1_PWM,0);
    analogWrite(MOTOR2_PWM,0);
    analogWrite(MOTOR3_PWM,0);

    //config digitalPIN
    pinMode(MOTOR1_IN_A, OUTPUT);
    pinMode(MOTOR1_IN_B, OUTPUT);
    pinMode(MOTOR2_IN_A, OUTPUT);
    pinMode(MOTOR2_IN_B, OUTPUT);
    pinMode(MOTOR3_IN_A, OUTPUT);
    pinMode(MOTOR3_IN_B, OUTPUT);
    pinMode(PIN_ACTUATOR, OUTPUT);
    //set motor without move
    digitalWrite(MOTOR1_IN_A,0);
    digitalWrite(MOTOR1_IN_B,0);
    digitalWrite(MOTOR2_IN_A,0);
    digitalWrite(MOTOR2_IN_B,0);
    digitalWrite(MOTOR3_IN_A,0);
    digitalWrite(MOTOR3_IN_B,0);
    digitalWrite(PIN_ACTUATOR,0);

    //Init time to sample
    prev_time_board =millis();

  #endif
}


void loop()
{
  #ifdef ESP32_38P
    time_board = millis();
    if(time_board-prev_time_board>=DT_BOARD)
    {
      prev_time_board=time_board;
      #ifdef PID_CONTROL
        PIDcompute(PWM_MAX);
      #endif
      #ifdef ON_OFF_CONTROL
        //OnOffCompute(PWM_MAX);
        uint16_t *ptnPWM = scara.compute(setMotor, med_ang_, PWM_motor);
        Serial.println(ptnPWM[0]);
        Serial.println(ptnPWM[1]);
        Serial.println(ptnPWM[2]);
        Serial.println("DATASSSSS");
        Serial.println(PWM_motor[0]);
        Serial.println(PWM_motor[1]);
        Serial.println(PWM_motor[2]);
      #endif

    }
      if (nh.connected()) 
        {
        
          pRobotPot.publish( &pot_msg );
          pRobotSensor.publish( &sensor_msg);
          #ifdef PID_CONTROL
            pRobotKp.publish(&kp_msg);
            pRobotKi.publish(&ki_msg);
            pRobotKd.publish(&kd_msg);
          #endif
          if (EmStop==0){
            if (PWM_motor[0]>0){
              digitalWrite(MOTOR1_IN_A,1);
              digitalWrite(MOTOR1_IN_B,0);
            }else if(PWM_motor[0]<0){
              digitalWrite(MOTOR1_IN_A,0);
              digitalWrite(MOTOR1_IN_B,1);
            }else{
              digitalWrite(MOTOR1_IN_A,0);
              digitalWrite(MOTOR1_IN_B,0);
            }
            if (PWM_motor[1]>0){
              digitalWrite(MOTOR2_IN_A,1);
              digitalWrite(MOTOR2_IN_B,0);
            }else if(PWM_motor[1]<0){
              digitalWrite(MOTOR2_IN_A,0);
              digitalWrite(MOTOR2_IN_B,1);
            }else{
              digitalWrite(MOTOR2_IN_A,0);
              digitalWrite(MOTOR2_IN_B,0);
            }
            if (PWM_motor[2]>0){
              digitalWrite(MOTOR3_IN_A,1);
              digitalWrite(MOTOR3_IN_B,0);
            }else if(PWM_motor[2]<0){
              digitalWrite(MOTOR3_IN_A,0);
              digitalWrite(MOTOR3_IN_B,1);
            }else{
              digitalWrite(MOTOR3_IN_A,0);
              digitalWrite(MOTOR3_IN_B,0);
            }
            #ifdef PID_CONTROL
              analogWrite(MOTOR1_PWM,abs(PWM_motor[0]));
              analogWrite(MOTOR2_PWM,abs(PWM_motor[1]));
              analogWrite(MOTOR3_PWM,abs(PWM_motor[2]));
            #endif
            #ifdef ON_OFF_CONTROL
              analogWrite(MOTOR1_PWM,PWM_VEL);
              analogWrite(MOTOR2_PWM,PWM_VEL);
              analogWrite(MOTOR3_PWM,PWM_VEL);
            #endif
            if (setAct == 1){
              digitalWrite(PIN_ACTUATOR,1);
            }else{
              digitalWrite(PIN_ACTUATOR,0);
            }
          }else{
            Serial.println("EMERGENCY BUTTOM PRESSED");
            analogWrite(MOTOR1_PWM,0);
            digitalWrite(MOTOR1_IN_A,0);
            digitalWrite(MOTOR1_IN_B,0);

            analogWrite(MOTOR2_PWM,0);
            digitalWrite(MOTOR2_IN_A,0);
            digitalWrite(MOTOR2_IN_B,0);

            analogWrite(MOTOR3_PWM,0);
            digitalWrite(MOTOR3_IN_A,0);
            digitalWrite(MOTOR3_IN_B,0);

            digitalWrite(PIN_ACTUATOR,0);
          }
          

        } else {
          Serial.println("Not Connected SERVER");
          analogWrite(MOTOR1_PWM,0);
          digitalWrite(MOTOR1_IN_A,0);
          digitalWrite(MOTOR1_IN_B,0);

          analogWrite(MOTOR2_PWM,0);
          digitalWrite(MOTOR2_IN_A,0);
          digitalWrite(MOTOR2_IN_B,0);

          analogWrite(MOTOR3_PWM,0);
          digitalWrite(MOTOR3_IN_A,0);
          digitalWrite(MOTOR3_IN_B,0);

          digitalWrite(PIN_ACTUATOR,0);
        }

        potMotor[0]=pot1.readVoltage();
        med_ang_[0]= map(potMotor[0],POT_MIN1,POT_MAX1,0,PWM_MAX);
        potMotor[1]=pot2.readVoltage();
        med_ang_[1]= map(potMotor[1],POT_MIN2,POT_MAX2,0,PWM_MAX);
        potMotor[2]=pot3.readVoltage();
        med_ang_[2]= map(potMotor[2],POT_MIN3,POT_MAX3,0,PWM_MAX);
        pot_msg.data=potMotor;

        sensorMotor[0]=sensor1.readVoltage();
        sensorMotor[1]=sensor2.readVoltage();
        sensorMotor[2]=sensor3.readVoltage();
        sensor_msg.data=sensorMotor;
        #ifdef PID_CONTROL
          kp_msg.data=kp_motor;
          ki_msg.data=ki_motor;
          kd_msg.data=kd_motor;
        #endif

        
        nh.spinOnce();
        delay(DT_BOARD/10);
   #endif
}
