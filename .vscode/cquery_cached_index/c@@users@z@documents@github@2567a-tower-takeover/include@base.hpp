#ifndef _BASE_H_
#define _BASE_H_

void brake();
void coast();
void resetDrive();
void resetALl();
void setSlant(int s);
void setSpeed(int speed);
bool isDriving();
void driveAsync(int sp);
void drive(int sp);
void turnAsync(int sp);
void turn(int sp);
void driveOP();

#endif
