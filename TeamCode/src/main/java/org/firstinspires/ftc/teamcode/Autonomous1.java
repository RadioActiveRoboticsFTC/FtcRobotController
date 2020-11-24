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

    // The BaseAutonomous class will have it's runOpsMode function called.  It will set up
    // basic stuff for us, but then it calls it's own runAutoOpMode, which does nothing.
    // We 'override' runAutoOpMode to do *something*
    @Override
    public void runAutoOpMode() {

        // driving straight this distance parks the robot directly under
        // the sky bridge
//        driveStraight(0.25, 2*12.0, 15.0);
//
//        spinLeftP(90, 0.5);

        // robot starts from starting line closest to the center,
        // backed up against wall, pointing straight

        // drive forward ? feet
        driveStraight(0.25, 1*12.0 + 6.0, 15.0);

        // spin right small amount
        spinRightP(-10, 0.5);

        // detect objects
        int numRings = 0;
//       while (opModeIsActive() {
        numRings = countRings(robot);
//        }
        telemetry.addData("num rings", numRings);
        telemetry.update();

        // all goals share this part of the path
        // spint right to 90.
        spinRightP(-90, 0.5);

        // go straight until we are aligned with A & B

        // 3 possible paths:

        if (numRings == 0) {
            // A:  go straight ? feet
        }

        if (numRings == 1) {
            // B:  go straight ? feed, spin to -90, go straight ? feet, sping back to zero
        }

        if (numRings == 4) {
            // c:  go straight, just further then A
        }

        // back up the appropriate distance to park





    }

    public int countRings(Robot2020 robot) {

        int numRings = 0;
        if (robot.tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    String label = recognition.getLabel();
                    if (label == "Single") numRings = 1;
                    if (label == "Quad") numRings = 4;
                }
                telemetry.update();
            }

        }
        return numRings;
    }
}
