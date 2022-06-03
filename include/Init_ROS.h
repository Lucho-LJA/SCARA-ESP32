// Set the connection to rosserial socket server
nh.getHardware()->setConnection(server, serverPort);
    
nh.initNode();
rpm_msg.data_length=4;
mpu_msg.data_length=6;
encoder_msg.data_length=4;
    

    // Start to be polite
nh.advertise(omni_rpm);
nh.advertise(omni_mpu);
nh.advertise(omni_encoder);
nh.subscribe(omni_setpoint);
nh.subscribe(omni_kp);
nh.subscribe(omni_ki);
nh.subscribe(omni_kd);
nh.subscribe(omni_mov);