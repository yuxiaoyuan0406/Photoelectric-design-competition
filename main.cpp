#include "opcv.h"
#include "motor.h"
#include <wiringPi.h>

#include <iostream>
/*************************************************************
 * motor0
 * pin_1 ==> 24 ==> OUT3
 * pin_2 ==> 25 ==> OUT4
 * channel ==> pca.0
 * 
 * motor1
 * pin_1 ==> 22 ==> OUT1
 * pin_2 ==> 23 ==> OUT2
 * channel ==> pca.1
 * 
 * motor2
 * pin_1 ==> 27 ==> OUT3
 * pin_2 ==> 28 ==> OUT4
 * channel ==> pca.2
 * 
 * 红外开关1，检测黑线
 * blue     ==> GND
 * brown    ==> +5V
 * black    ==> 29
 * 
 * 红外开关2，水平检测障碍
 * blue     ==> GND
 * brown    ==> +5V
 * black    ==> 26
 * 
*************************************************************/

#define MIN_SPEED 0.14 //minimum speed
#define MAX_SPEED 0.70 //maximum speed

#define FULL_IMAGE 640 //image width
#define HALF_WIDTH 200 //half of the tolerate width

#define down 29  //look down
#define front 26 //look forward

int isLightKilled = wiringPiSetup(); //if found the black line then make it 1
//also call wiringPiSetup before pca
int isLightNear; //if the light is nearby the make it 1
//double useMaxSpeed = MAX_SPEED;

pca myPca(0x40, 7);
opcv camera;
motor myMotor[3];
servo myServo(myPca, 3);

double speed[3] = {0, 0, 0};

void writeSpeed()
{
    for (int i = 0; i < 3; i++)
        myMotor[i].setSpeed(speed[i]);
}

void turn(double duty)
{
    speed[0] = duty;
    speed[1] = duty;
    speed[2] = 0;
    writeSpeed();
}

void stopMotors()
{
    /*
    unsigned int delayTime = (unsigned(speed[0]) + unsigned(speed[1])) / 2.0 * 0.9 * 130;
    speed[0] = (2.0 * (speed[0] > 0) - 1.0) * -.9;
    speed[1] = (2.0 * (speed[1] > 0) - 1.0) * -.9;
    speed[2] = 0;
    writeSpeed();
    delay(delayTime);
    for (int i = 0; i < 3; ++i)
        speed[i] = 0;
    writeSpeed();
    */

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
            speed[j] = -.9;
        writeSpeed();
        delay(15);
        for (int j = 0; j < 3; ++j)
            speed[j] = .9;
        writeSpeed();
        delay(15);
    }

    for (int i = 0; i < 3; ++i)
        speed[i] = 0;
    writeSpeed();
}

void findLight(double duty)
{
    turn(duty);
    //delay(30);
    //stopMotors(motors);
    int light;
    if (isLightKilled == 1 || isLightNear == 0)
        return;
    while (camera.readx() == -1 && isLightKilled == 0 && isLightNear == 0)
        ;
    stopMotors();
    light = camera.readx();
    if (light >= FULL_IMAGE / 2 - HALF_WIDTH && light <= FULL_IMAGE / 2 - HALF_WIDTH && isLightKilled == 0 && isLightNear == 0)
    {
        return;
    }
    else if (light < FULL_IMAGE / 2 - HALF_WIDTH && isLightKilled == 0 && isLightNear == 0)
    {
        findLight(MIN_SPEED);
        return;
    }
    else if (light > FULL_IMAGE / 2 + HALF_WIDTH && isLightKilled == 0 && isLightNear == 0)
    {
        findLight(-MIN_SPEED);
        return;
    }
}

void goStraight(double straightSpeed)
{
    int x = camera.readx();
    if (x == -1 && isLightKilled == 0 && isLightNear == 0)
        return;
    double spinSpeed = double(FULL_IMAGE / 2 - x) / double(FULL_IMAGE / 2) * 0.4 * straightSpeed;
    speed[0] = straightSpeed + spinSpeed / 2.0;
    speed[1] = -straightSpeed + spinSpeed / 2.0;
    speed[2] = spinSpeed;
    writeSpeed();
}

void killLight()
{
    if (digitalRead(down) == 1)
        isLightKilled = 1;
}

void slowDown()
{
    if (digitalRead(front) == 0)
        isLightNear = 1;
}

void speedUp()
{
    if (digitalRead(front) == 1)
        isLightNear = 0;
}

void backup(double duty)
{
    speed[0] = duty;
    speed[1] = duty;
    speed[2] = 0;
    writeSpeed();
}

void setup()
{
    myServo.write(0);

    pinMode(down, INPUT);
    pinMode(front, INPUT);
    wiringPiISR(down, INT_EDGE_RISING, &killLight);
    wiringPiISR(front, INT_EDGE_FALLING, &slowDown);
//    wiringPiISR(front, INT_EDGE_RISING, &speedUp);

    //    myMotor[0].setup(24, 25, myPca, 0);
    //    myMotor[1].setup(22, 23, myPca, 1);
    //    myMotor[2].setup(27, 28, myPca, 2);
    isLightKilled = 0;
    isLightNear = 0;

    myMotor[0].setup(25, 24, myPca, 0);
    myMotor[1].setup(23, 22, myPca, 1);
    myMotor[2].setup(28, 27, myPca, 2);
}

int main()
{
    using namespace std;

    setup();

    while (1)
    {
        while (isLightKilled == 0 && isLightNear == 0)
        {
            int light = camera.readx();
            if (isLightKilled == 0 && isLightNear == 0 && light == -1)
                turn(MIN_SPEED);
            else if (isLightKilled == 0 && isLightNear == 0)
                goStraight(MAX_SPEED);
        }
        while (isLightNear == 1)
        {
            cout << "light nearby" << endl;
            if (isLightKilled == 0)
                goStraight(0.3);
            else
            {
                stopMotors();
                myServo.write(130);
                delay(1200);
                myServo.write(0);
                delay(600);
                backup(MAX_SPEED / 2.0);
                delay(500);
                stopMotors();
                isLightKilled = isLightNear = 0;
            }
        }
    }
    /*
    for(int i = 0; i < 3; i++)
    {
        speed[i] = .3;
    }
    writeSpeed();
    delay(300);
    stopMotors();
    */
    /*
    while(1)
    {
        cout << "input speed" << endl;
        cin >> speed[0];
        speed[1] = -speed[0];
        speed[2] = 0;
        writeSpeed();
    }*/
    return 0;
}
