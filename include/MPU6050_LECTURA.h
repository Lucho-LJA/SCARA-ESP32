#ifndef _MPU6050_LECTURA_H_
    #define _MPU6050_LECTURA_H_
    #include "config.h"
    #include <string>
    #include "MPU6050_6Axis_MotionApps20.h"

    // uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
    // (in degrees) calculated from the quaternions coming from the FIFO.
    // Note that Euler angles suffer from gimbal lock (for more info, see
    // http://en.wikipedia.org/wiki/Gimbal_lock)
    #define OUTPUT_READABLE_EULER


    // uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
    // components with gravity removed. This acceleration reference frame is
    // not compensated for orientation, so +X is always +X according to the
    // sensor, just without the effects of gravity. If you want acceleration
    // compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
    #define OUTPUT_READABLE_REALACCEL

    MPU6050 mpu;
    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



    void Iniciar_MPU6050()
    {
        #ifndef _INIT_SERIAL_
            #define _INIT_SERIAL_
            Serial.begin(115200);
        #endif
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
            #define _INIT_WIRE_
            Wire.begin();
            Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
            Fastwire::setup(400, true);
        #endif
        mpu.initialize();    //Iniciando el sensor

        // verify connection
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
        
        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXAccelOffset(1157);
        mpu.setYAccelOffset(-1629);
        mpu.setZAccelOffset(1244); // 16388 factory default for my test chip
        mpu.setXGyroOffset(-241);
        mpu.setYGyroOffset(45);
        mpu.setZGyroOffset(-1);
    

        // make sure it worked (returns 0 if so)
        if (devStatus == 0) 
        {
            // Calibration Time: generate offsets and calibrate our MPU6050
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            mpu.PrintActiveOffsets();
            // turn on the DMP, now that it's ready
            mpu.setDMPEnabled(true);

            mpuIntStatus = mpu.getIntStatus();

            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serial.println(F("DMP ready! Waiting for first interrupt..."));
            dmpReady = true;

            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
        }
    }

        

    void Leer_mpu6050() 
    {

        // if programming failed, don't try to do anything
        if (!dmpReady) 
        {
            Error_sistema="SIN CONEXION DE MPU6050";
            return;
        }
         // read a packet from FIFO
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
        { 
            #ifdef OUTPUT_READABLE_EULER
                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetEuler(euler, &q);
                MPU_motor[3]=euler[0] * 180/M_PI;
                MPU_motor[4]=euler[1] * 180/M_PI;
                MPU_motor[5]=euler[2] * 180/M_PI;
            #endif
            #ifdef OUTPUT_READABLE_REALACCEL
                // display real acceleration, adjusted to remove gravity
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                MPU_motor[0]=aaReal.x*(double) ACCEL_SCALE*G_TO_ACCEL;
                MPU_motor[1]=aaReal.y*(double) ACCEL_SCALE*G_TO_ACCEL;
                MPU_motor[2]=aaReal.z*(double) ACCEL_SCALE*G_TO_ACCEL;
        }
        #endif
        
    }

#endif