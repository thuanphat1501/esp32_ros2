
#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 25 // In1 (3,4)
  #define LEFT_MOTOR_BACKWARD  2  // In2 (1,2)
  #define RIGHT_MOTOR_FORWARD  26 //In1
  #define LEFT_MOTOR_FORWARD   15 //In2
  #define RIGHT_MOTOR_ENABLE 12 // chân Enable B
  #define LEFT_MOTOR_ENABLE 13 //chân Enable A

  // #define RIGHT_MOTOR_IN1 25 //
  // #define LEFT_MOTOR_IN2  2
  // #define RIGHT_MOTOR_IN1  26
  // #define LEFT_MOTOR_IN2   15
  // #define EN_A 12  // ĐIỀU KHIỂN PWM LEFT
  // #define EN_B 13   //ĐIỀU KHIỂN PWM RIGHT 
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);