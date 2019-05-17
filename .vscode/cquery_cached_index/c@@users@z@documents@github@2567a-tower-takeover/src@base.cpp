#include "main.h"

#define MAX 127;
int driveTarget;
int turnTarget;
int slant = 0;
int maxSpeed = MAX

//defining motors & gyro
Motor leftDrive(11, MOTOR_GEARSET_18, 0,  MOTOR_ENCODER_DEGREES);
Motor leftDrive1(12, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_DEGREES);
Motor rightDrive(13, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_DEGREES);
Motor rightDrive1(14, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_DEGREES);
ADIGyro gyro ('A');

/************************Basic Control************************************/
void left(int vel)
{
  leftDrive.move(vel);
  leftDrive1.move(vel);
}

void right(int vel)
{
  rightDrive.move(vel);
  rightDrive1.move(vel);
}

/************************Modifications************************************/

void resetDrive()
{
  leftDrive.tare_position();
  leftDrive1.tare_position();
  rightDrive.tare_position();
  rightDrive1.tare_position();
}

void resetAll()
{
  resetDrive();
  maxSpeed = MAX;
  slant = 0;
}

void setSlant(int s)
{
  /*(if(mirror)
  {
    s = -s;
  }
   slant = s; */
}

void setSpeed(int speed)
{
  maxSpeed = speed;
}

void brake()
{
  leftDrive.set_brake_mode(MOTOR_BRAKE_HOLD);
  leftDrive1.set_brake_mode(MOTOR_BRAKE_HOLD);
  rightDrive.set_brake_mode(MOTOR_BRAKE_HOLD);
  rightDrive1.set_brake_mode(MOTOR_BRAKE_HOLD);
}

void coast()
{
  leftDrive.set_brake_mode(MOTOR_BRAKE_COAST);
  leftDrive1.set_brake_mode(MOTOR_BRAKE_COAST);
  rightDrive.set_brake_mode(MOTOR_BRAKE_COAST);
  rightDrive1.set_brake_mode(MOTOR_BRAKE_COAST);
}

/************************Acceleration************************************/

void leftSlew(int slewSpeed)
{
  int step;
  static int speed = 0;
  if(abs(speed) < abs(slewSpeed))
  {
    step = 5;
  }
  else
  {
    step = 256; // no slew
  }


  if(speed < slewSpeed - step)
  {
    speed += step;
  }
  else if(speed > slewSpeed + step)
  {
    speed -= step;
  }
  else
  {
    speed = slewSpeed;
  }

   left(speed);
}

void rightSlew(int slewSpeed)
{
  int step;
  static int speed = 0;
  if(abs(speed) < abs(slewSpeed))
  {
    step = 5;
  }
  else
  {
    step = 256; // no slew
  }


  if(speed < slewSpeed - step)
  {
    speed += step;
  }
  else if(speed > slewSpeed + step)
  {
    speed -= step;
  }
  else
  {
    speed = slewSpeed;
  }

   right(speed);
}

/************************Feedback************************************/

bool isDriving()
{
  static int count = 0;
  static int last = 0;
  static int lastTarget = 0;

  int leftPos = leftDrive.get_position();
  int rightPos = rightDrive.get_position();

  int curr = (abs(leftPos) + abs(rightPos))/2;
  int thresh = 5;
  int target = driveTarget;

  if(abs(last-curr) < thresh)
    count++;
  else
    count = 0;

  if(target != lastTarget)
    count = 0;

  lastTarget = target;
  last = curr;

  //not driving if we haven't moved
  if(count > 8)
    return false;
  else
    return true;
}

/************************Auton Commands************************************/
void driveAsync(int sp)
{
  resetDrive();
  driveTarget = sp;

}

void drive(int sp)
{
  driveAsync(sp);
  while(isDriving());
}

void turnAsync(int sp)
{
  resetDrive();
  turnTarget = sp;
}

void turn(int sp)
{
  turnAsync(sp);
  while(isDriving());
}

/************************Tasks************************************/

void driveTask(void *parameter)
{
  int prevError = 0;

  while(1){
    delay(20);

    int sp = driveTarget;

    double kp = 0;
    double kd = 0;

    //read sensor
    int sv = leftDrive.get_position();

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

      int posDiff = fabs(rightDrive.get_position()) - fabs(leftDrive.get_position()); // check for right encoder difference
      		int posCorrection = signed(posDiff)*speed*.1; 					// posCorrection = 10% drivePower in the direction of posDiff


    //set motors
    leftSlew(speed + posCorrection - slant);
    rightSlew(speed + slant);
  }
}

void turnTask(void *parameter)
{
  int prevError = 0;

  while(1){
    delay(20);

    int sp = driveTarget;

    double kp = 0;
    double kd = 0;

    //read sensor
    int sv = gyro.get_value();

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


    //set motors
    leftSlew(speed - slant);
    rightSlew(speed + slant);
  }
}

/************************Driver Control************************************/

void driveOP()
{
  leftDrive.move(controller.get_analog(ANALOG_LEFT_Y));
  leftDrive1.move(controller.get_analog(ANALOG_LEFT_Y));
  rightDrive.move(controller.get_analog(ANALOG_RIGHT_Y));
  rightDrive1.move(controller.get_analog(ANALOG_RIGHT_Y));
}
