   
#ifdef ARDUINO_ENC_COUNTER


  #define LEFT_ENC_PIN_A 19   // pin 16
  #define LEFT_ENC_PIN_B 21   // pin 17

  #define RIGHT_ENC_PIN_A 32  //pin A4
  #define RIGHT_ENC_PIN_B 33   //pin A5
  #define ENC_COUNT_REV 200;
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

void EncoderInit(void);
// void ENCRA(void);
// void ENCRB(void);
void L_ENA();
void R_ENA();
