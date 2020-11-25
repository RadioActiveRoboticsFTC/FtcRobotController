package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

// Directions:
// Place robot against wall, facing the sky bridge, either side.  Have
// the front of the robot almost under the sky bridge.

// Points:
// 5 points for parking the robot directly under the sky bridge

// This is an example of creating an autonomous mode based off of
// BaseAutonomous (we 'extend' it)

@Autonomous(name="Autonomous1")
public class Autonomous1 extends BaseAutonomous {

    // This is working for starting on left red start line, with no second wobble in the way.
    // But if you keep the second wobble in it's start position, it will get in the way.

    // The BaseAutonomous class will have it's runOpsMode function called.  It will set up
    // basic stuff for us, but then it calls it's own runAutoOpMode, which does nothing.
    // We 'override' runAutoOpMode to do *something*
    @Override
    public void runAutoOpMode() {

        float ts = (float) 15.0;

        // driving straight this distance parks the robot directly under
        // the sky bridge
//        driveStraight(0.25, 2*12.0, 15.0);
//
//        spinLeftP(90, 0.5);

        // robot starts from starting line closest to the center,
        // backed up against wall, pointing straight

        // drive forward ? feet
        driveStraight(0.25, 1*12.0 + 10.0, 15.0);

        // spin right small amount
        spinRightP(-10, 0.5);

        sleep(2000);
        // detect objects
        int numRings = 0;
//       while (opModeIsActive() {
        numRings = countRings(robot);
        sleep(500);
        int numRings2 = countRings(robot);
//        }
        telemetry.addData("num rings", numRings);
        telemetry.addData("num rings2", numRings2);
        telemetry.update();

        // all goals share this part of the path
        // spint right to 90.
        spinRightP(-90, 0.5);

        // go straight until we are aligned with A & B
        driveStraight(0.5, 2*12.0 + 0.0, 15.0);


        // 3 possible paths:

        if (numRings == 0) {
            // A:  go straight ? feet
            spinLeftP(10, 0.25);
            driveStraight(0.5, 4*12 + 6, 15.0);
            // drop the wobble
            robot.wobbleServo.setPosition(1.0);
            // we are already on the line!
        }

        if (numRings == 1) {
            // B:  go straight ? feed, spin to -90, go straight ? feet, sping back to zero
//            spinLeftP(18, 0.25);
//
//            driveStraight(0.5, 7*12, 15.0);

            spinLeftP(0, 0.25);
            driveStraight(0.5, 1*12, ts);
            spinLeftP( 28, 0.25);
            driveStraight(0.5, 6*12, ts);

            // drop the wobble
            robot.wobbleServo.setPosition(1.0);
            sleep(2000);
            spinRightP(0, 0.25);
            float inches = -2*12;
            encoderDrive(-.3, inches, inches, 10, false);
;
        }

        if (numRings == 4) {
            // c:  go straight, just further then A
            spinLeftP(5, 0.25);
            driveStraight(0.5, 8.4*12, 15.0);
            // drop the wobble
            robot.wobbleServo.setPosition(1.0);
            sleep(2000);
            spinRightP(0, 0.25);
            float inches = -4*12;
            encoderDrive(-.3, inches, inches, 10, false);



        }

    }


}
