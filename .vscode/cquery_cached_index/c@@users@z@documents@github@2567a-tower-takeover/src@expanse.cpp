#include "main.h"

#define MAX 127;
int liftTarget;
int maxSpeed = MAX

//defining motors & gyro
Motor leftLift(11, MOTOR_GEARSET_18, 0,  MOTOR_ENCODER_DEGREES);
Motor rightLift(12, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_DEGREES);

void liftAsync(int sp)
{
  resetDrive();
  liftTarget = sp;
}

void lift(int sp)
{
  liftAsync(sp);
  while(isDriving());
}

/************************Tasks************************************/

void liftTask(void *parameter)
{
  int prevError = 0;

  while(1){
    delay(20);

    int sp = liftTarget;

    double kp = 0;
    double kd = 0;

    //read sensor
    int sv = leftLift.get_position();

    //calculate speed
    int error = sp-sv;
    int derivative = error - prevError;
    prevError = error;
    int speed = error*kp + derivative*kd;

    //setting max and min speed
    if(speed > maxSpeed)
      speed = maxSpeed;
    if(speed < -maxSpeed)
      speed = -maxSpeed;
    if(speed > 0 && speed < 20)
      speed = 20;
    if(speed < 0 && speed >-20)
      speed = -20;

      int posDiff = fabs(rightLift.get_position()) - fabs(leftLift.get_position()); // check for right encoder difference
      		int posCorrection = signed(posDiff)*speed*.1; 					// posCorrection = 10% drivePower in the direction of posDiff


    //set motors
    leftSlew(speed + posCorrection);
    rightSlew(speed);
