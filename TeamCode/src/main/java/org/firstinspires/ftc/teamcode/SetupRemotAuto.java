package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


// This is an example of creating an autonomous mode based off of
// BaseAutonomous (we 'extend' it)

@Autonomous(name="SetupRemoteAuto")
public class SetupRemotAuto extends BaseAutonomous {

    // This is for holding the wobble goal for autonomous.
    // Run this before your match.


    // The BaseAutonomous class will have it's runOpsMode function called.  It will set up
    // basic stuff for us, but then it calls it's own runAutoOpMode, which does nothing.
    // We 'override' runAutoOpMode to do *something*
    @Override
    public void runAutoOpMode() {

        // open servo
        sleep(1000);

        // TODO: use a robot function for this
        robot.wobbleServo.setPosition(1.0);

        sleep(1000);

        robot.wobbleServo.setPosition(0);


    }


}
