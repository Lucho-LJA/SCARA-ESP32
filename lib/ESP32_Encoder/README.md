# ESP32_Encoder Library

Encoder counts pulses from quadrature encoded signals, which are commonly available from rotary knobs, motor or shaft sensors and other position sensors.
Use of Arduino framwork and ESP32S-38 pins.
The code was used with Platformio.
Use:
    To inicializated the encoder
        <wheel_diametre must to be in metres. If use others units, the result will be in these unit, consider it when use other funcions>
        MOTOR[4]={pinA, pinB, Rpm_max, ppr_encoder, wheel_diametre}
        Encoder encoder_1(MOTOR);
    To get velocity on RPM
        rpm_motor=wncoder_1.getRPM();
    To get actual position in unit of wheel_diametre
        position=encoder_1.read();
    To get actual position and reset (position in unit of wheel_diametre)
        position=encoder_1.readAndReset();
    To reset
        encoder_1.reset();
    To write the actual position, use unit of wheel_diametre
        encoder_1.write(position);
            
