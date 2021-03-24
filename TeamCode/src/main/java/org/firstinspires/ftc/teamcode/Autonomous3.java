package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

// Directions:
// put robot directly over left red line, with back to wall

@Autonomous(name="Autonomous3")
public class Autonomous3 extends BaseAutonomous {


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
        spinRightP(-45, 0.5);

        sleep(2000);
        // detect objects
        int numRings = 0;
//       while (opModeIsActive() {
        numRings = countRings(robot);
        //sleep(500);
        //int numRings2 = countRings(robot);
//        }
        telemetry.addData("num rings", numRings);
        //telemetry.addData("num rings2", numRings2);
        telemetry.update();

        // all goals share this part of the path
//        // spint right to 90.
//        spinRightP(-90, 0.5);
//
//        // go straight until we are aligned with A & B
//        driveStraight(0.5, 2*12.0 + 0.0, 15.0);


        // 3 possible paths:

        if (numRings == 0) {
            robot.shooterMotor.setPower(0.55);
            // don't got straight so we are in the middle of target B
            spinLeftP(5.0, 0.25);
//            driveStraight(0.8, 5.5*12 , 20.0);

            // let's instead go up to the shoot line, shoot, then continue

            driveStraight(0.75, (3*12)-7, 20);
            spinRightP(-5, 0.25);
            shootRing();
            driveStraight(0.75, 7, 20);
            spinRightP(-80, 0.25);
            robot.shooterMotor.setPower(0);
            driveStraight(0.5, 3.5*12, 20);
            robot.wobbleServo.setPosition(1.0);
            sleep(500);
            robot.setPower(-0.25,-0.5);
            sleep(2000);

            // turn backwards to park on line
            // A:  go straight ? feet
//            spinLeftP(10, 0.25);
//            driveStraight(0.5, 4*12 + 6, 15.0);
//            // drop the wobble
//            robot.wobbleServo.setPosition(1.0);
            // we are already on the line!
        }

        if (numRings == 1) {
            robot.shooterMotor.setPower(0.55);
            // don't got straight so we are in the middle of target B
            spinLeftP(5.0, 0.25);
//            driveStraight(0.8, 5.5*12 , 20.0);

            // let's instead go up to the shoot line, shoot, then continue

            driveStraight(0.75, (3*12)-7, 20);
            spinRightP(-5, 0.25);
            shootRing();
            driveStraight(0.75, 7, 20);
            spinLeftP(5.0, 0.25);
            robot.shooterMotor.setPower(0);
            driveStraight(0.75, 2.5*12, 20.0);

            spinRightP(-180, 0.75);
            // drop the wobble
            robot.wobbleServo.setPosition(1.0);
            sleep(500);

            driveStraight(0.75, 1*12, 20.0);


            // B:  go straight ? feed, spin to -90, go straight ? feet, sping back to zero
//            spinLeftP(18, 0.25);
//
//            driveStraight(0.5, 7*12, 15.0);

//            spinLeftP(0, 0.25);
//            driveStraight(0.5, 1*12, ts);
//            spinLeftP( 28, 0.25);
//            driveStraight(0.5, 6*12, ts);
//
//            // drop the wobble
//            robot.wobbleServo.setPosition(1.0);
//            sleep(2000);
//            spinRightP(0, 0.25);
//            float inches = -2*12;
//            encoderDrive(-.3, inches, inches, 10, false);
//            ;
        }

        if (numRings == 4) {
            // c:  go straight, just further then A
//            spinLeftP(5, 0.25);
//            driveStraight(0.5, 8.4*12, 15.0);
//            // drop the wobble
//            robot.wobbleServo.setPosition(1.0);
//            sleep(2000);
//            spinRightP(0, 0.25);
//            float inches = -4*12;
//            encoderDrive(-.3, inches, inches, 10, false);
            robot.shooterMotor.setPower(.55);
            spinLeftP(0.0, 0.25);
            driveStraight(0.75, (3*12)-7, 20);
            spinRightP(-5, 0.25);
            shootRing();
            spinLeftP(0.0, 0.25);
            robot.shooterMotor.setPower(0.0);
            driveStraight(0.8, (7.5*12)-(3*12)+5, 20.0);
            spinRightP(-80, 0.75);
            driveStraight(0.75, 1.75*12, 20.0);
            spinRightP(-180, 0.75);

            // drop the wobble
            robot.wobbleServo.setPosition(1.0);
            driveStraight(0.75, 3*12, 20.0);
        }


    }


}



