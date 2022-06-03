// interrupt pins for known boards

// Teensy (and maybe others) define these automatically
#if !defined(CORE_NUM_INTERRUPT)

  #if defined(ESP32)

    #define CORE_NUM_INTERRUPT  40 
    #define CORE_INT0_PIN		0
    #define CORE_INT1_PIN		1
    #define CORE_INT2_PIN		2
    #define CORE_INT3_PIN		3
    #define CORE_INT4_PIN		4
    #define CORE_INT5_PIN		5
    // GPIO6-GPIO11 are typically used to interface with the flash memory IC on 
    // esp32, so we should avoid adding interrupts to these pins.
    #define CORE_INT12_PIN	12
    #define CORE_INT13_PIN	13
    #define CORE_INT14_PIN	14
    #define CORE_INT15_PIN	15
    #define CORE_INT16_PIN        16
    #define CORE_INT17_PIN        17
    #define CORE_INT18_PIN        18
    #define CORE_INT19_PIN        19
    #define CORE_INT21_PIN        21
    #define CORE_INT22_PIN        22
    #define CORE_INT23_PIN        23
    #define CORE_INT25_PIN        25
    #define CORE_INT26_PIN        26
    #define CORE_INT27_PIN        27
    #define CORE_INT32_PIN        32
    #define CORE_INT33_PIN        33
    #define CORE_INT34_PIN        34
    #define CORE_INT35_PIN        35
    #define CORE_INT36_PIN        36
    #define CORE_INT39_PIN        39
  #endif
#endif

#if !defined(CORE_NUM_INTERRUPT)
  #error "Interrupts are unknown for this board, please add to this code"
#endif
#if CORE_NUM_INTERRUPT <= 0
  #error "Encoder requires interrupt pins, but this board does not have any :("
  #error "You could try defining ENCODER_DO_NOT_USE_INTERRUPTS as a kludge."
#endif

