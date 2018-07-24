#include "opcv.h"
#include "motor.h"
#include <wiringPi.h>
#include <cmath>

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
 * 红外开关0
 * blue     ==> GND
 * brown    ==> +5V
 * black    ==> 1
 * 
 * 红外开关1
 * blue     ==> GND
 * brown    ==> +5V
 * black    ==> 29
 * 
*************************************************************/

//speed to use(100%)
#define MIN_SPEED 0.1285 //minimum speed
#define MID_SPEED 0.60
#define MAX_SPEED 0.80 //maximum speed

//light in view(px)
#define FULL_WIDTH 640 //image width
#define FULL_HIGHT 480
#define HALF_WIDTH 300 //half of the tolerate width
#define BREAK_WIDTH 65
#define BREAK_HIGHT 250

//servo mode(degrees)
#define servoBegin 90
#define servoWork 180

const int lightSwitch[] =
    {1, 4, 5, 6, 26};

int isLightKilled = wiringPiSetup(); //if found the black line then make it 1
//also call wiringPiSetup before pca
int lightNearby = 0;
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
    std::cout << "slide aside..." << std::endl;
    char dir;
    if (digitalRead(lightSwitch[4]) == 0)
    {
        if (digitalRead(lightSwitch[0]) == 0)
        {
            dir = 1;
        }
        else
        {
            dir = -1;
        }
    }
    else
    {
        dir = 1;
    }
    speed[0] = dir * -MAX_SPEED / 2.5; //speed adjust
    speed[1] = speed[0];
    speed[2] = dir * MAX_SPEED;
    writeSpeed();
    delay(500);   //wait for this time
    stopMotors(); //stop
}

void findLight(double duty)
{
    std::cout << "enter findLight func" << std::endl;
    int light;
    if (isLightKilled == 1)
        return;
    light = camera.readx(lightWidth, lightHight);
    if (isLightKilled == 1)
        return;
    lastSeeLight = light > 320 ? -1 : 1;
    duty *= lastSeeLight;
    turn(duty);
    //delay(30);
    //stopMotors(motors);
    while (camera.readx(lightWidth) == -1 && isLightKilled == 0)
        std::cout << "finding..." << std::endl;
    if (light < FULL_WIDTH / 2 - HALF_WIDTH && light > FULL_WIDTH / 2 - HALF_WIDTH && isLightKilled == 0)
    {
        std::cout << "light found!\nfirst adjust" << std::endl;
        findLight(MIN_SPEED);
    }
    stopMotors();
    return;
}

void farFieldAdjust()
{
    std::cout << "enter farFieldAdjust func" << std::endl;
    int x = camera.readx(lightWidth, lightHight);
    double straightSpeed;
    double modifiedSpeed;
    if (x == -1 || isLightKilled == 1 || lightWidth > BREAK_WIDTH || lightHight > BREAK_HIGHT)
        return;
    else if (isLightKilled == 0 && lightWidth <= BREAK_WIDTH)
    {
        /*straightSpeed = MAX_SPEED;
        modifiedSpeed = 0.5 * 0.4;
        double spinSpeed = sin(double(FULL_WIDTH / 2 - x) / double(FULL_WIDTH / 2.0) * M_PI_2) * modifiedSpeed;
        speed[0] = straightSpeed + spinSpeed * 0.9;
        speed[1] = -straightSpeed + spinSpeed * 0.9;
        speed[2] = spinSpeed * 1.7;*/
        straightSpeed = MAX_SPEED;
        modifiedSpeed = 0.1;
        if (abs(FULL_WIDTH / 2 - x) > 100)
            modifiedSpeed = 0.135;
        else if (abs(FULL_WIDTH / 2 - x) > 200)
            modifiedSpeed = 0.15;
        double spinSpeed = double(x < FULL_WIDTH / 2 ? 1 : -1) * modifiedSpeed;
        speed[0] = straightSpeed + spinSpeed;
        speed[1] = -straightSpeed + spinSpeed;
        speed[2] = spinSpeed;
    }
    writeSpeed();
}

void middleFieldAdjust()
{
    std::cout << "enter middleFieldAdjust func" << std::endl;
    int x = camera.readx(lightWidth, lightHight);
    double straightSpeed;
    double modifiedSpeed;
    if (x == -1 || isLightKilled == 1 || lightWidth > BREAK_WIDTH || lightHight > BREAK_HIGHT)
        return;
    else if (isLightKilled == 0 && lightWidth <= 60)
    {
        /*    straightSpeed = MID_SPEED;
        modifiedSpeed = 0.5 * 0.4;
        double spinSpeed = sin(double(FULL_WIDTH / 2 - x) / double(FULL_WIDTH / 2.0) * M_PI_2) * 1.0 * modifiedSpeed;
        speed[0] = straightSpeed + spinSpeed * 0.6;
        speed[1] = -straightSpeed + spinSpeed * 0.6;
        speed[2] = spinSpeed * 0.5;
        */
        straightSpeed = MID_SPEED;
        modifiedSpeed = 0.07;
        if (abs(FULL_WIDTH / 2 - x) > 100)
            modifiedSpeed = 0.1;
        else if (abs(FULL_WIDTH / 2 - x) > 200)
            modifiedSpeed = 0.13;
        else if (abs(FULL_WIDTH / 2 - x) > 280)
            modifiedSpeed = 0.2;
        double spinSpeed = double(x < FULL_WIDTH / 2 ? 1 : -1) * modifiedSpeed;
        speed[0] = straightSpeed + spinSpeed;
        speed[1] = -straightSpeed + spinSpeed;
        speed[2] = spinSpeed;
    }
    writeSpeed();
    std::cout << "middleFieldAdjust speed wrote" << std::endl;
}

void nearFieldAdjust()
{
    std::cout << "enter nearFieldAdjust func" << std::endl;
    while (isLightKilled == 0)
    {
        std::cout << "adjusting..." << std::endl;
        int light = camera.readx(lightWidth);
        double straightSpeed;
        double modifiedSpeed;
        if (light == -1 || isLightKilled == 1 || lightWidth <= BREAK_WIDTH)
            return;
        else if (isLightKilled == 0 && lightWidth > BREAK_WIDTH)
        {
            straightSpeed = 0.17;
            modifiedSpeed = 1.3 * straightSpeed;
            double spinSpeed = double(FULL_WIDTH / 2 - light) / double(FULL_WIDTH / 2.0 - 0.0) * 0.4 * 0.9 * /*straightSpeed */ modifiedSpeed;
            speed[0] = straightSpeed + spinSpeed;
            speed[1] = -straightSpeed + spinSpeed;
            speed[2] = spinSpeed;
        }
        writeSpeed();
    }
    std::cout << "nearFieldAdjust done!" << std::endl;
}

void slightlyAdjust()
{
    std::cout << "enter slightlyAdjust func" << std::endl;
    if (digitalRead(lightSwitch[1]) == 0)
    {
        std::cout << "left switch triggered" << std::endl;
        if (digitalRead(lightSwitch[3]) == 0)
        {
            std::cout << "right switch triggered" << std::endl;
            stopMotors();
            return;
        }
        else
        {
            std::cout << "right switch is not triggered\nadjusting..." << std::endl;
            speed[0] = MIN_SPEED;
            speed[1] = MIN_SPEED;
            speed[2] = MIN_SPEED;
        }
    }
    else
    {
        std::cout << "left switch is not triggered" << std::endl;
        if (digitalRead(lightSwitch[3]) == 0)
        {
            std::cout << "right switch is triggered\nadjusting..." << std::endl;
            speed[0] = -MIN_SPEED;
            speed[1] = -MIN_SPEED;
            speed[2] = -MIN_SPEED;
        }
        else
        {
            std::cout << "right switch is not triggered" << std::endl;
            if (camera.readx(lightWidth) == -1 || lightWidth < 200)
            {
                std::cout << "no light in view or too small" << std::endl;
                slideAside();
                delay(300);
                stopMotors();
                isLightKilled = 0;
                std::cout << "reset isLightKilled" << std::endl;
                return;
            }
            else
            {
                std::cout << "light in view\ngoing forward..." << std::endl;
                speed[0] = MIN_SPEED * 1.03;
                speed[1] = -MIN_SPEED * 1.03;
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

    if (digitalRead(lightSwitch[0] == 0 || lightSwitch[2] == 0 || lightSwitch[4] == 0))
    {
        std::cout << "far light switch triggered!\nlightNearby set to 1" << std::endl;
        lightNearby = 1;
    }
    if (digitalRead(lightSwitch[1]) == 0 || digitalRead(lightSwitch[3]) == 0)
    {
        std::cout << "light switch triggered!\nisLightKilled set to 1" << std::endl;
        isLightKilled = 1;
    }
}

void backup(double duty)
{
    std::cout << "backing up..." << std::endl;
    speed[0] = -duty;
    speed[1] = duty;
    speed[2] = 0;
    writeSpeed();
}

void setup()
{
    std::cout << "initializing..." << std::endl;
    myServo.write(servoBegin);
    for (int i = 0; i < 5; i++)
    {
        pinMode(lightSwitch[i], INPUT);
        wiringPiISR(lightSwitch[i], INT_EDGE_FALLING, &killLight);
    }
    /*    wiringPiISR(lightSwitch[1], INT_EDGE_FALLING, &killLight);
    wiringPiISR(lightSwitch[3], INT_EDGE_FALLING, &killLight);
*/

    isLightKilled = 0;

    myMotor[0].setup(23, 22, myPca, 1);
    myMotor[1].setup(25, 24, myPca, 2);
    myMotor[2].setup(28, 27, myPca, 0);
}

int main()
{
    using namespace std;

    setup();

    while (1)
    {
        while (isLightKilled == 0)                            //when light switch is not triggered
        {                                                     //stay in this loop
            int light = camera.readx(lightWidth, lightHight); //read a coordinate of the light in view
            if (isLightKilled == 0 && light == -1)
                findLight(0.1);
            else if (isLightKilled == 0 && lightNearby == 0 && lightWidth <= BREAK_WIDTH && lightHight <= 60 && lightHight > 60)
                farFieldAdjust();
            else if (isLightKilled == 0 && lightNearby == 0 && lightWidth <= BREAK_WIDTH && lightHight <= BREAK_HIGHT)
                middleFieldAdjust();
            else if (isLightKilled == 0 && lightWidth > BREAK_WIDTH)
            {
                stopMotors();
                nearFieldAdjust();
            }
            else if (lightNearby == 1 && lightWidth <= BREAK_WIDTH / 0.75)
            {
                stopMotors();
                slideAside();
                delay(500);
                stopMotors();
                lightNearby = 0;
            }
        }
        if (camera.readx(lightWidth) == -1 || lightWidth <= 100)
        { //if light is too small or no light in view but the light switch is triggered
            cout << "no light in view" << endl;
            //slide aside
            stopMotors(); //stop motor first
            //stopMotors();     //
            slideAside();
            delay(300);
            stopMotors();
            isLightKilled = 0; //initiate isLightKilled
        }
        else
        {
            cout << "light nearby" << endl;
            stopMotors();
            slightlyAdjust();
            stopMotors();
            cout << "slightlyAdjust done!\nkilling light" << endl;
            myServo.write(servoWork);
            delay(650);
            myServo.write(servoBegin);
            delay(650);
            cout << "light killed!" << endl;
            backup(MAX_SPEED);
            delay(300);
            stopMotors();
            cout << "backing up done!\ninitializing isLightKilled" << endl;
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
