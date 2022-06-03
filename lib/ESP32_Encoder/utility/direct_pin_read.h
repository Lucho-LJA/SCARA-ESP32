#ifndef direct_pin_read_h
    #define direct_pin_read_h

    /* ESP32  Arduino (https://github.com/espressif/arduino-esp32) */
    #if defined(ESP32)

        #define IO_REG_TYPE			            uint32_t
        #define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
        #define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
        #define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)
    #endif

#endif
