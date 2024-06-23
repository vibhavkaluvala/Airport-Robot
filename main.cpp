#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHServo.h>
#include <FEHMotor.h>
#include <FEHRCS.h>
#include <math.h>
#include <FEHBattery.h>

#define RADIUS 1.25
#define PI 3.14
#define TRANSITIONS 318
#define WIDTH 9

FEHMotor left_drive(FEHMotor::Motor0, 9.0);
FEHMotor right_drive(FEHMotor::Motor1, 9.0);

FEHServo robot_arm(FEHServo::Servo0);
FEHServo drakinator(FEHServo::Servo1);

AnalogInputPin cds_sensor(FEHIO::P0_1);
DigitalEncoder right_encoder(FEHIO::P1_0);
DigitalEncoder left_encoder(FEHIO::P1_3);

/**
 * waitForLight makes the robot wait to initiate movement until a change of light happens
 * @pre the CdS cell sensor is working and in bank 0 port 1
 * @post The CdS cell sensor has noticed a considerable change of light
 */
void waitForLight()
{
    float lastReading = cds_sensor.Value();
    float change = 0;
    while (change <= 1){
        float currentReading = cds_sensor.Value();
        change = abs(lastReading-currentReading);
        lastReading = currentReading;
        Sleep(0.2);
    }
    // while (cds_sensor.Value() >= 1.5)
    // {
    //     Sleep(0.2);
    // }
}
/**
 * driveFront drives the robot forward the distance the user tells it too.
 * @pre left motor is in port 2_3 and right motor is in port 2_0
 * @param requestedDistance - the distance in inches to travel
 * @post The robot has driven forwards a positive distance if requestedDistance > 0 otherwise backwards
 */
void drive(float requestedDistance, bool isRamp)
{
    /*
     * Sets motor percentage to 30 if the requested distance is positive otherwise sets it to -30.
     */
    int motorPercentage = 30;
    if (isRamp){
        motorPercentage += 15;
    }
    if (requestedDistance < 0)
    {
        motorPercentage = -motorPercentage;
    }

    motorPercentage = motorPercentage*11.5/Battery.Voltage();
    /*
     * Sets distnace to the absolute value of requestedDistance
     */
    float distance = abs(requestedDistance);
    /*
     * Tolerance is how often you want the robot to correct itself.
     * Then num_iterations finds how many times it will run through based on the distance
     * and tolerance. After there is still some leftover distance the robot needs to cover along
     * with its corresponding rotations.
     */
    float leftCorrection = 1.02;
    int tolerance = 1;
    int num_iterations = distance / tolerance;
    float leftover_distance = distance - num_iterations * tolerance;
    float tolerance_rotations = tolerance * TRANSITIONS / (2 * PI * RADIUS);
    float leftover_rotations = leftover_distance * TRANSITIONS / (2 * PI * RADIUS);

    for (int i = 0; i < num_iterations; i++)
    {
        /*
         * Resets both econder counts before each while loop to track proper distance
         */
        left_encoder.ResetCounts();
        right_encoder.ResetCounts();
        /*
         * Starts both motors to go at the percentage defined above.
         */
        left_drive.SetPercent(motorPercentage);
        right_drive.SetPercent(motorPercentage);
        /*
         * This will run until both encoders have counted enough rotations for the tolerance
         */
        while (left_encoder.Counts() < tolerance_rotations * leftCorrection || right_encoder.Counts() < tolerance_rotations)
        {
            /*
             * Checks if we need the left motor to stop otherwise we check if the right motor needs to stop
             */
            if (left_encoder.Counts() >= tolerance_rotations * leftCorrection)
            {
                left_drive.Stop();
            }
            else if (right_encoder.Counts() >= tolerance_rotations)
            {
                right_drive.Stop();
            }
        }
        /*
         * Both motors should then be stopped after the while loop
         */
        left_drive.Stop();
        right_drive.Stop();
    }
    /*
     * We reset the counts again to finish the leftover distance and then start both motors again.
     */
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    left_drive.SetPercent(motorPercentage);
    right_drive.SetPercent(motorPercentage);
    /*
     * This will run until both encoders have counted enough rotations for the tolerance
     */
    while (left_encoder.Counts() < leftover_rotations * leftCorrection && right_encoder.Counts() < leftover_rotations)
    {
        /*
         * Checks if we need the left motor to stop otherwise we check if the right motor needs to stop
         */
        if (left_encoder.Counts() * leftCorrection >= leftover_rotations)
        {
            left_drive.Stop();
        }
        else if (right_encoder.Counts() >= leftover_rotations)
        {
            right_drive.Stop();
        }
    }
    /*
     * Finally we stop both motors.
     */
    left_drive.Stop();
    right_drive.Stop();
}

/**
 * The turn method ensures that the inputted motor and econder turns the robot the correct amount of degrees
 * @pre A functionting motor and ecncoder is being used
 * @param motor - the motor to turn with
 * @param encoder - the encoder to track the distacne travelled
 * @param degrees - the amount of degrees the robot should turn
 * @post The robot has turned the user given degrees with the provided motor and encoder.
 */
void turn(FEHMotor motor, DigitalEncoder encoder, float degrees)
{
    encoder.ResetCounts();
    int motorPercentage =  35*11.5/Battery.Voltage();
    if (degrees > 0)
    {
        motor.SetPercent(motorPercentage);
    }
    else
    {
        motor.SetPercent(-motorPercentage);
    }

    // 10 should be bot width
    float distance = (2 * PI * WIDTH) / (360 / degrees);
    float num_rotations = abs(distance) / (2 * PI * RADIUS);

    while (encoder.Counts() < TRANSITIONS * num_rotations)
    {
        // LCD.WriteLine(encoder.Counts());
        // LCD.WriteLine(num_rotations);
    }

    motor.Stop();
}

void turnAroundCenter(float degrees)
{
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    int motorPercentage =  35*11.5/Battery.Voltage();

    if (degrees > 0)
    {
        left_drive.SetPercent(-motorPercentage);
        right_drive.SetPercent(motorPercentage);
    }
    else
    {
        left_drive.SetPercent(motorPercentage);
        right_drive.SetPercent(-motorPercentage);
        degrees *= -1;
    }

    // 10 should be bot width
    float distance = (2 * PI * WIDTH) / (360 / degrees);
    float num_rotations = distance / (2 * PI * RADIUS);

    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    
    while (right_encoder.Counts() < TRANSITIONS * num_rotations)
    {
        // LCD.WriteLine(num_rotations);
    }

    left_drive.Stop();
    right_drive.Stop();

}

/**
 * @pre a functioning CdS cell sensor.
 * @post the value returned is true if the light is red, false if the light is blue.
 */
bool isRedOrBlue()
{
    // left_drive.SetPercent(8);
    // right_drive.SetPercent(8);
    // float timeOld = TimeNow();
    // int counter = 0;
    // while (cds_sensor.Value() > 2.0 && TimeNow() - timeOld < 8.0){
    //     if (counter = 0){
    //         left_drive.SetPercent(12);
    //         right_drive.SetPercent(8);
    //         counter = 1;
    //     } else {
    //         left_drive.SetPercent(12);
    //         right_drive.SetPercent(8);
    //         counter = 0;
    //     }
    //     Sleep(0.15);
    // }
    // left_drive.Stop();
    // right_drive.Stop();
    bool result = false;
    if (cds_sensor.Value() > 1.2)
    {
        result = true;
    }
    drive(-5, false);
    return result;
}

void pressAppropriateButton(bool isRed)
{
    // Rotates to face the center
    if (isRed)
    {
        // Hits blue button
        turnAroundCenter(-32);
        drive(-8, false);
        robot_arm.SetDegree(70);
    }
    else
    {
        drive(-3, false);
        turnAroundCenter(-32);
        drive(-8, false);
        // drive(5);
        // turnAroundCenter(35);
        // drive(5);
    }
}

void flipCorrectSwitch(int num){
    int endDistance = -18;
    /*
    * Drive to the correct lever
    */
    switch (num){
        case 0:
            turnAroundCenter(35);
            drive(1.8, false);
            break;
        case 1:
            turnAroundCenter(35);
            drive(1.8, false);
            break;
        case 2:
            turnAroundCenter(35);
            drive(1.8, false);
            break;
    }
    /*
    * Put the arm in the down position
    */

    robot_arm.SetDegree(125);
    Sleep(2.0);
    robot_arm.SetDegree(70);
    Sleep(3.0);
    drive(-4.7, false);
    robot_arm.SetDegree(140);

    /* Start driving, turns 45 deg, then inches to left tank. Turns 90 deg, arm is up, so drive forward, then set degree to 90 to turn the switch.  */
    drive(5.1, false);
    robot_arm.SetDegree(120);
    Sleep(1.0);
    robot_arm.SetDegree(130);
    Sleep(1.0);

    /* Turns and backs up into wall.*/
    turnAroundCenter(34);
    robot_arm.SetDegree(70);
    drive(endDistance, false);

    /* Turns towards ramp. */
    drive(2, false);
    turnAroundCenter(38);
}

void flipPassport(){
    robot_arm.SetDegree(180);
    drive(10, false);
    drive(-0.5, false);
    robot_arm.SetDegree(30);
    Sleep(2.0);
    robot_arm.SetDegree(80);
    drive(-1.5, false);
}

void celebrate(){
    float time = TimeNow();
    while (TimeNow() <= 10+time){
        drive(2, true);
        drakinator.SetDegree(180);
        robot_arm.SetDegree(180);
        Sleep(1000);
        drive(-2, true);
        drakinator.SetDegree(0);
        robot_arm.SetDegree(0);
        Sleep(1000);
   }
}

int main(void)
{
    /*
    * Calibration values for Servo DO NOT DELETE
    */
    robot_arm.SetMin(500);
    robot_arm.SetMax(2500);
    drakinator.SetMin(500);
    drakinator.SetMax(2500);
    /*
    * Initializes RCS so that we can get the proper lever
    */
   robot_arm.SetDegree(70);
    RCS.InitializeTouchMenu("E4UjirFqb");
    waitForLight();
    /*
    * Code used to get to the area to flip the first switch
    */
    drive(-2.5, false);
    drive(18, false);
    turn(right_drive,right_encoder, 35);
    drive(-1.8, false);
    flipCorrectSwitch(RCS.GetCorrectLever());

    /*
    * Drives over to drop off the suitcase
    */
    drive(19.5, true);
    turn(left_drive,left_encoder,77);
    drive(2.5, false);
    drakinator.SetDegree(180);
    Sleep(0.2);
    drakinator.Off();

    /* Backs up from luggage area to square off. */
    drive(-2,false);
    drive(-17.5, false);
    drive(24,false);

    /* Turns to back up into ticket booth area. Then squares off right before checking for ticket booth light. */
    turnAroundCenter(-19);
    drive(-32, false);
    
    /* This will hopefully make robot wiggle over light and read the correct value. */
    bool color = isRedOrBlue();
    LCD.WriteLine(color);

    /* If the robot is able to read the value, then have it square off again before getting to passport. */
    
    // Work on getting to passport
    drive(7.75, false);
    turnAroundCenter(20);
    flipPassport();
    pressAppropriateButton(color);

    /* Driving towards end button*/
    drive(13, false);
    turn(right_drive, right_encoder, 45);
    drive(7, false);
    turn(left_drive, left_encoder, 45); 
    drive(20, true);
    celebrate();
    drive(15, true);
    turn(right_drive, right_encoder, 55);
    drive(-2, false);
    drive(5, true);
}
