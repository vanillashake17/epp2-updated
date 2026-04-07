#include <AFMotor.h>
// Direction values
typedef enum dir
{
  STOP,
  GO,
  BACK,
  CCW,
  CW
} dir;

// Motor control
#define FRONT_LEFT   4 // M4 on the driver shield
#define FRONT_RIGHT  1 // M1 on the driver shield
#define BACK_LEFT    3 // M3 on the driver shield
#define BACK_RIGHT   2 // M2 on the driver shield

AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);

void move(int speed, int direction)
{
  
  motorFL.setSpeed(speed);
  motorFR.setSpeed(speed);
  motorBL.setSpeed(speed);
  motorBR.setSpeed(speed);

  switch(direction)
    {
      case BACK:
        motorFL.run(BACKWARD);
        motorFR.run(BACKWARD);
        motorBL.run(FORWARD);
        motorBR.run(FORWARD); 
      break;
      case GO:
        motorFL.run(FORWARD);
        motorFR.run(FORWARD);
        motorBL.run(BACKWARD);
        motorBR.run(BACKWARD); 
      break;
      case CW:
        // Turn right: left wheels forward, right wheels backward
        motorFL.run(FORWARD);
        motorFR.run(FORWARD);
        motorBL.run(FORWARD);
        motorBR.run(FORWARD);
      break;
      case CCW:
        // Turn left: left wheels backward, right wheels forward
        motorFL.run(BACKWARD);
        motorFR.run(BACKWARD);
        motorBL.run(BACKWARD);
        motorBR.run(BACKWARD);
      break;
      case STOP:
      default:
        motorFL.run(RELEASE);
        motorFR.run(RELEASE);
        motorBL.run(RELEASE);
        motorBR.run(RELEASE); 
    }
}

void forward(int speed)
{
  move(speed, GO);
}

void backward(int speed)
{
  move(speed, BACK);
}

void ccw(int speed)
{
  move(speed, CCW);
}

void cw(int speed)
{
  move(speed, CW);
}

void stop()
{
  move(0, STOP);
}

