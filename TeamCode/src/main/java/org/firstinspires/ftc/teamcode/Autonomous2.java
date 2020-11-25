package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Directions:
// Place robot against wall, facing the sky bridge, either side.  Have
// the front of the robot almost under the sky bridge.

// Points:
// 5 points for parking the robot directly under the sky bridge

// This is an example of creating an autonomous mode based off of
// BaseAutonomous (we 'extend' it)

@Autonomous(name="Autonomous2")
public class Autonomous2 extends BaseAutonomous {

    // This is working for starting on right red start line, with a second wobble in the way.


    // The BaseAutonomous class will have it's runOpsMode function called.  It will set up
    // basic stuff for us, but then it calls it's own runAutoOpMode, which does nothing.
    // We 'override' runAutoOpMode to do *something*
    @Override
    public void runAutoOpMode() {

        // timeout (secs)
        float ts = (float) 15.0;

        // approach the rings
        driveStraight(0.25, 1*12.0 + 10.0, 15.0);

        // spin left small amount to see rings
        spinLeftP(5, 0.5);

        // settle before looking
        sleep(2000);

        // detect objects
        int numRings = 0;

        numRings = countRings(robot);

        telemetry.addData("num rings", numRings);
        telemetry.update();

        // all goals share this part of the path
        // spint right to 90.
        spinRightP(-45, 0.5);

        // TBF: take out more common stuff from below?

        // 3 possible paths:
        if (numRings == 0) {
            // A:  go straight ? feet
            driveStraight(0.5, 1*12 + 2, ts);

            // straighten out
            spinLeftP(0, 0.25);

            // go to target A
            driveStraight(0.5, 3*12 + 6, 15.0);
            // drop the wobble
            robot.wobbleServo.setPosition(1.0);
            // we are already on the line!
        }

        if (numRings == 1) {
            // B:
            // just like A, but then turn into B

            driveStraight(0.5, 1*12 + 2, ts);

            // straighten out
            spinLeftP(0, 0.25);

            // go to target A
            driveStraight(0.5, 2*12 + 6, 15.0);

            // turn into B
            spinLeftP(40, 0.25);
            driveStraight(0.5, 3*12, ts);

            // drop the wobble
            robot.wobbleServo.setPosition(1.0);

            // go park
            sleep(500);
            spinRightP(0, 0.25);
            float inches = -1*12;
            encoderDrive(-.8, inches, inches, 10, false);
;
        }

        if (numRings == 4) {
            // c:  go straight, just further then A

            driveStraight(0.5, 1*12 + 1, ts);

            // straighten out
            spinLeftP(0, 0.25);

            // go to target C
            driveStraight(0.5, 7*12 + 6, 15.0);

            // drop the wobble
            robot.wobbleServo.setPosition(1.0);

            // go park
            sleep(1000);
            float inches = -4*12;
            encoderDrive(-.8, inches, inches, 10, false);



        }

    }


}
