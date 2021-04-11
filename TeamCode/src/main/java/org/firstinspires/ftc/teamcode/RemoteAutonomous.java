package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

// this opmode runs through our whole autonomous program for a remote competition.
// we shoot rings, drop wobble goal in location specified by number of rings detected,
// and we park

// Directions:
// put robot directly over left red line, with back to wall

@Autonomous(name="RemoteAutonomous")
public class RemoteAutonomous extends BaseAutonomous {


    // The BaseAutonomous class will have it's runOpsMode function called.  It will set up
    // basic stuff for us, but then it calls it's own runAutoOpMode, which does nothing.
    // We 'override' runAutoOpMode to do *something*
    @Override
    public void runAutoOpMode() {

        float ts = (float) 15.0;

        // hold on to the wobble
        robot.graspWobble();

        // drive forward
        driveStraight(0.25, 1*12.0 + 10.0, 15.0);

        // spin right small amount
        spinRightP(-45, 0.5);

        sleep(2000);

        // detect objects
        int numRings = countRings(robot);

        telemetry.addData("num rings", numRings);
        telemetry.update();

        //varible for shooting power
        double shootingPower = 0.50; //.55 was dramatically overshooting .5 barely undershooting
        // 3 possible paths:

        if (numRings == 0) {
            robot.shooterMotor.setPower(shootingPower);
            // don't got straight so we are in the middle of target B
            spinLeftP(5.0, 0.25);
//            driveStraight(0.8, 5.5*12 , 20.0);

            // let's instead go up to the shoot line, shoot, then continue
            driveStraight(0.75, (3*12)-7, 20);
            spinRightP(-5, 0.25);
            shootRing();
            sleep(1000);
            shootRing();

            // go to zone A to drop the wobble
            driveStraight(0.75, 7, 20);
            spinRightP(-80, 0.25);
            robot.shooterMotor.setPower(0);
            driveStraight(0.5, 3.5*12, 20);
            robot.dropWobble();
            sleep(500);

            // go park robot over parking line
            robot.setPower(-0.25,-0.5);
            sleep(2000);

        }

        if (numRings == 1) {
            robot.shooterMotor.setPower(shootingPower);
            // don't got straight so we are in the middle of target B
            spinLeftP(5.0, 0.25);
//            driveStraight(0.8, 5.5*12 , 20.0);

            // let's instead go up to the shoot line, shoot, then continue
            driveStraight(0.75, (3*12)-7, 20);
            spinRightP(-5, 0.25);
            shootRing();
            sleep(1000);
            shootRing();

            // got to zone B to drop wobble
            driveStraight(0.75, 7, 20);
            spinLeftP(5.0, 0.25);
            robot.shooterMotor.setPower(0);
            driveStraight(0.75, 2.5*12, 20.0);
            spinRightP(-180, 0.75);
            // drop the wobble
            robot.dropWobble();
            sleep(500);

            // park!
            driveStraight(0.75, 1*12, 20.0);

        }

        if (numRings == 4) {

            // this makes us drive to the white line and shoot twice
            robot.shooterMotor.setPower(shootingPower);
            spinLeftP(0.0, 0.25);
            driveStraight(0.75, (3*12)-7, 20);
            spinRightP(-5, 0.25);
            shootRing();
            sleep(1000);
            shootRing();

            // go to zone C to drop wobble
            spinLeftP(0.0, 0.25);
            robot.shooterMotor.setPower(0.0);
            driveStraight(1, (7.5*12)-(3*12)+5, 20.0);
            spinRightP(-70, 1);
            driveStraight(1, 1.75*12, 20.0);
            spinRightP(-180, 1);

            // drop the wobble
            robot.dropWobble();

            // go park
            driveStraight(0.75, 3*12, 20.0);
        }


    }


}



