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

//speed to use(100%)
#define MIN_SPEED 0.122 //minimum speed
#define MAX_SPEED 0.60  //maximum speed

//light in view(px)
#define FULL_IMAGE 640 //image width
#define HALF_WIDTH 300 //half of the tolerate width
#define BREAK_WIDTH 65

//servo mode(degrees)
#define servoBegin 90
#define servoWork 180

const int lightSwitch[] =
    {1, 29};

int isLightKilled = wiringPiSetup(); //if found the black line then make it 1
//also call wiringPiSetup before pca
int lightWidth = 0; //the width of the light in view
int lightHight = 0; //the hight coordinate in view

int lastSeeLight = 1;

pca myPca(0x40, 7);
opcv camera;
motor myMotor[3];
servo myServo(myPca, 3);

double speed[3] =
    {0, 0, 0};

void writeSpeed()
{
    for (int i = 0; i < 3; i++)
        myMotor[i].setSpeed(speed[i]);
}

void turn(double duty)
{
    speed[0] = duty;
    speed[1] = duty;
    speed[2] = duty;
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

void slideAside()
{
    speed[0] = -MAX_SPEED * sqrt(3.0) / 2; //speed adjust
    speed[1] = speed[0];
    speed[2] = MAX_SPEED;
    writeSpeed();
    delay(300);   //wait for this time
    stopMotors(); //stop
}

void findLight(double duty)
{
    int light;
    if (isLightKilled == 1)
        return;
    light = camera.readx(lightWidth);
    if (isLightKilled == 1)
        return;
    lastSeeLight = light > 320 ? -1 : 1;
    duty *= lastSeeLight;
    turn(duty);
    //delay(30);
    //stopMotors(motors);
    while (camera.readx(lightWidth) == -1 && isLightKilled == 0)
        ;
    if (light < FULL_IMAGE / 2 - HALF_WIDTH && light > FULL_IMAGE / 2 - HALF_WIDTH && isLightKilled == 0)
    {
        findLight(MIN_SPEED);
    }
    stopMotors();
    return;
}

void goStraight()
{
    int x = camera.readx(lightWidth);
    /*    double straightSpeed = 0.1735;
    double modifiedSpeed = 1.3 * straightSpeed;
    if (x == -1 && isLightKilled == 0)
        return;
    if (isLightKilled == 0 && lightWidth < 70)
    {
        straightSpeed = MAX_SPEED;
        modifiedSpeed = 0.7 * straightSpeed;
    }
    double spinSpeed = double(FULL_IMAGE / 2 - x) / double(FULL_IMAGE / 2.0 - 0.0) * 0.4 * 0.9 * straightSpeed  modifiedSpeed;*/
    double straightSpeed;
    double modifiedSpeed;
    if (x == -1 || isLightKilled == 1 || lightWidth > BREAK_WIDTH)
        return;
    else if (isLightKilled == 0 && lightWidth <= 60)
    {
        straightSpeed = MAX_SPEED;
        modifiedSpeed = 0.5 * straightSpeed;
        double spinSpeed = double(FULL_IMAGE / 2 - x) / double(FULL_IMAGE / 2.0 - 0.0) * 0.65 * /*straightSpeed */ modifiedSpeed;
        speed[0] = straightSpeed + spinSpeed * 1.5;
        speed[1] = -straightSpeed + spinSpeed * 1.5;
        speed[2] = spinSpeed * 1.6;
    }
    writeSpeed();
}

void nearFieldAdjust()
{
    while (isLightKilled == 0)
    {
        int light = camera.readx(lightWidth);
        double straightSpeed;
        double modifiedSpeed;
        if (light == -1 || isLightKilled == 1 || lightWidth <= BREAK_WIDTH)
            return;
        else if (isLightKilled == 0 && lightWidth > BREAK_WIDTH)
        {
            straightSpeed = 0.1735;
            modifiedSpeed = 1.5 * straightSpeed;
            double spinSpeed = double(FULL_IMAGE / 2 - light) / double(FULL_IMAGE / 2.0 - 0.0) * 0.4 * 0.9 * /*straightSpeed */ modifiedSpeed;
            speed[0] = straightSpeed + spinSpeed / 1.2;
            speed[1] = -straightSpeed + spinSpeed / 1.2;
            speed[2] = spinSpeed / 1.2;
        }
        writeSpeed();
    }
}

void slightlyAdjust()
{
    if (digitalRead(lightSwitch[0]) == 0)
    {
        if (digitalRead(lightSwitch[1]) == 0)
        {
            stopMotors();
            return;
        }
        else
        {
            speed[0] = MIN_SPEED;
            speed[1] = MIN_SPEED;
            speed[2] = MIN_SPEED;
        }
    }
    else
    {
        if (digitalRead(lightSwitch[1]) == 0)
        {
            speed[0] = -MIN_SPEED;
            speed[1] = -MIN_SPEED;
            speed[2] = -MIN_SPEED;
        }
        else
        {
            if (camera.readx(lightWidth) == -1 || lightWidth < 200)
            {
                slideAside();
                isLightKilled = 0;
                return;
            }
            else
            {
                speed[0] = MIN_SPEED;
                speed[1] = -MIN_SPEED;
                speed[2] = 0;
            }
        }
    }
    writeSpeed();
    slightlyAdjust();
    return;
}

void killLight()
{
    if (digitalRead(lightSwitch[0]) == 0 || digitalRead(lightSwitch[1]) == 0)
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
    for (int i = 0; i < 2; i++)
    {
        pinMode(lightSwitch[i], INPUT);
        wiringPiISR(lightSwitch[i], INT_EDGE_FALLING, &killLight);
    }

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
        while (isLightKilled == 0)                //when light switch is not triggered
        {                                         //stay in this loop
            int light = camera.readx(lightWidth); //read a coordinate of the light in view
            if (isLightKilled == 0 && light == -1)
                findLight(0.1);
            else if (isLightKilled == 0 && lightWidth <= BREAK_WIDTH)
                goStraight();
            else if (isLightKilled == 0 && lightWidth > BREAK_WIDTH)
            {
                stopMotors();
                nearFieldAdjust();
            }
        }
        if (camera.readx(lightWidth) == -1)
        {
            isLightKilled = 0;
        }
        else if (lightWidth < 100) //if light is too small but the light switch is triggered
        {                          //slide aside
            stopMotors();          //stop motor first
            stopMotors();          //
            slideAside();
            isLightKilled = 0; //initiate isLightKilled
        }
        else
        {
            stopMotors();
            slightlyAdjust();
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
