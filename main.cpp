#include "opcv.h"
#include "motor.h"
#include <wiringPi.h>

#include <iostream>
/*************************************************************
 * motor0
 * pin_1 ==> 23 ==> OUT1
 * pin_2 ==> 22 ==> OUT2
 * channel ==> pca.1
 * 
 * motor1
 * pin_1 ==> 27 ==> OUT3
 * pin_2 ==> 28 ==> OUT4
 * channel ==> pca.0
 * 
 * motor2
 * pin_1 ==> 24 ==> OUT3
 * pin_2 ==> 25 ==> OUT4
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

#define MIN_SPEED 0.122 //minimum speed
#define MAX_SPEED 0.60  //maximum speed

#define FULL_IMAGE 640 //image width
#define HALF_WIDTH 200 //half of the tolerate width

#define servoBegin 90
#define servoWork 180

#define down 29  //look down
#define front 26 //look forward

int isLightKilled = wiringPiSetup(); //if found the black line then make it 1
//also call wiringPiSetup before pca
int lightWidth = 0; //the width of the light in view

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
    for (int i = 0; i < 3; ++i)
        speed[i] = -myMotor[i].currentSpeed;
    writeSpeed();
    delay(100);
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
    if (isLightKilled == 1)
        return;
    while (camera.readx(lightWidth) == -1 && isLightKilled == 0)
        ;
    stopMotors();
    light = camera.readx(lightWidth);
    if (light >= FULL_IMAGE / 2 - HALF_WIDTH && light <= FULL_IMAGE / 2 - HALF_WIDTH && isLightKilled == 0)
    {
        return;
    }
    else if (light < FULL_IMAGE / 2 - HALF_WIDTH && isLightKilled == 0)
    {
        findLight(MIN_SPEED);
        return;
    }
    else if (light > FULL_IMAGE / 2 + HALF_WIDTH && isLightKilled == 0)
    {
        findLight(-MIN_SPEED);
        return;
    }
}

void goStraight()
{
    int x = camera.readx(lightWidth);
    double straightSpeed = 0.1735;
    //double modifiedSpeed = 0.9 * straightSpeed;
    if (x == -1 && isLightKilled == 0)
        return;
    if (isLightKilled == 0 && lightWidth < 20)
    {
        straightSpeed = MAX_SPEED;
        //        modifiedSpeed = 0.9;
    }
    double spinSpeed = double(FULL_IMAGE / 2 - x) / double(FULL_IMAGE / 2.0 - 0.0) * 0.4 * 0.9 * straightSpeed /*modifiedSpeed*/;
    speed[0] = straightSpeed + spinSpeed / 1.5;
    speed[1] = -straightSpeed + spinSpeed / 1.5;
    speed[2] = spinSpeed * 1.6;
    writeSpeed();
}

void killLight()
{
    if (digitalRead(down) == 1)
        isLightKilled = 1;
}

void backup(double duty)
{
    speed[0] = -duty;
    speed[1] = duty;
    speed[2] = 0;
    writeSpeed();
}

void setup()
{
    myServo.write(servoBegin);

    pinMode(down, INPUT);
    pinMode(front, INPUT);
    wiringPiISR(down, INT_EDGE_RISING, &killLight);
    //    wiringPiISR(front, INT_EDGE_RISING, &speedUp);

    //    myMotor[0].setup(24, 25, myPca, 0);
    //    myMotor[1].setup(22, 23, myPca, 1);
    //    myMotor[2].setup(27, 28, myPca, 2);
    isLightKilled = 0;

    myMotor[0].setup(23, 22, myPca, 1);
    myMotor[1].setup(28, 27, myPca, 0);
    myMotor[2].setup(25, 24, myPca, 2);
}

int main()
{
    using namespace std;

    setup();
    
    while (1)
    {
        while (isLightKilled == 0)
        {
            int light = camera.readx(lightWidth);
            if (isLightKilled == 0 && light == -1)
                turn(MIN_SPEED);
            else if (isLightKilled == 0)
                goStraight();
        }
        if (camera.readx(lightWidth) == -1)
        {
            isLightKilled = 0;
        }
        else if (lightWidth < 100)
        {
            //            isLightKilled = 0;
            stopMotors();
            stopMotors();
            speed[0] = -MAX_SPEED * sqrt(3.0) / 2.0;
            speed[1] = speed[0];
            speed[2] = -speed[0];//MAX_SPEED;
            writeSpeed();
            delay(300);
            stopMotors();
            isLightKilled = 0;
        }
        else
        {
            stopMotors();
            myServo.write(servoWork);
            delay(1200);
            myServo.write(servoBegin);
            delay(650);
            backup(MAX_SPEED);
            delay(400);
            stopMotors();
            isLightKilled = 0;
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
    while (1)
    {
        for (int i = 0; i < 3; i++)
            speed[i] = 0;
        int i = 0;
        cout << "input motor" << endl;
        cin >> i;
        cout << "input speed" << endl;
        cin >> speed[i];
        writeSpeed();
    }*/
    /*    for (int i = 0; i < 3; i++)
        speed[i] = 0;
    writeSpeed();
    return 0;*/
}
