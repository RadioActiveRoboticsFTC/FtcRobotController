/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


@TeleOp(name="Working Remote Control", group="Linear Opmode")
public class DriverControl extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();

    private final int GOTO_DONE = 0;
    private final int GOTO_ALIGN1 = 1;
    private final int GOTO_MOVEX = 2;
    private final int GOTO_MOVEY = 3;
    private final int GOTO_ALIGN2 = 4;

    @Override
    public void runOpMode() {

        // create our robot object so we have access to motors, etc.
        Robot2020 robot = new Robot2020();

        // setup the robot motors via the configuration file
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        int goToState = GOTO_DONE;

        // this is so that the motor will have already started up when we start the match so that
        // in the match we do not have to wait for it to start up
        robot.shooterMotor.setPower(0.0);
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            double pos = robot.leftDrive.getCurrentPosition();
            //telemetry.addData("pos", pos);
            //robotMotion(robot);

            if (gamepad1.a) {
                // robot vision
                robotVision(robot);
                //goToState = goToTarget(robot, goToState);

            } else {
                // moves the robot
//                telemetry.addData("right joy", gamepad1.right_stick_x);
//                telemetry.addData("a pressed", gamepad1.a);
                robotMotion(robot);

                goToState = GOTO_DONE;
            }

            //countRings(robot);
//            robot.shooterMotor.setPower(gamepad2.right_stick_y*0.5);
            double shootPower = gamepad2.right_stick_y*0.5;
            robot.shooterMotor.setPower(shootPower);

            if (gamepad2.right_bumper) {
                // shoot the rings!
//                robot.shootRing();

//                robot.wallServo.setPosition(0.5);
                robot.wallServo.setPosition(0.15);
                 sleep(500);
//        wait(500);
                robot.pusherServo.setPosition(0.65  );
            } else {
                // put shooter stuff back in oringinal position
//                robot.resetShooter();
                robot.wallServo.setPosition(0.0);
                sleep(500);
//        wait(500);
                robot.pusherServo.setPosition(0.85);
            }
            // this lefts us aim our shooter with the up/down motion

            if(gamepad2.left_bumper){
                robot.lowerIntakeMotor.setPower(1);
                robot.upperIntakeMotor.setPower(1);
            }
            else{
                robot.lowerIntakeMotor.setPower(0);
                robot.upperIntakeMotor.setPower(0);
            }
            double shooterAnglePower = 0.3 * gamepad2.left_stick_y;
            robot.shooterAngleMotor.setPower(shooterAnglePower);



            idle();

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("shoot power", shootPower);

            telemetry.update();
        }
    }

    public void robotVision(Robot2020 robot) {

        OpenGLMatrix lastLocation = null;
        float mmPerInch        = 25.4f;

        // check all the trackable targets to see which one (if any) is visible.
        boolean targetVisible = false;
        for (VuforiaTrackable trackable : robot.allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                //telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
         //If we dont know where we are bail

         //Check the State
        //The states are align, X movement, Y motion, Last alignment
        //If state is align check gyro sensor
        //If not pointed straight turn entill we are
        //If pointed straight change state to "X Movement"

        //If "X Movement" check the distance from target
        //If distance from target is greater than a talerance move
        //If close enough to target transion to "Y Movement"


        // Provide feedback as to where the robot is located (if we know).
        if ((targetVisible) && (lastLocation != null))  {
            // express position (translation) of robot in inches.

            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            float x = translation.get(0) / mmPerInch;
            float y = translation.get(1) / mmPerInch;
            float z = translation.get(2) / mmPerInch;

            float xTarget = 10;
            float yTarget = -10;
            float xd = xTarget-x;
            float yd = yTarget-y;
            double distance = Math.sqrt((xd*xd) + (yd*yd));
            telemetry.addData("distance",distance);

            float power = (float) 0.0;
            if (Math.abs(x - xTarget) > 6) {
                // move robot closer
                if (x > xTarget) {
                    // move backwared
                    power = (float) -0.5;
                } else {
                    // mover forward
                    power = (float) 0.5;
                }

            }
            robot.setPower(power);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            telemetry.update();
        }
//        else {
//            telemetry.addData("Visible Target", "none");
//        }
    }

    // function for moving the robot
    public void robotMotion(Robot2020 robot) {

        //grab needed values from gamepad1
        float gear = gamepad1.right_trigger;
        float strafeTrigger = gamepad1.left_trigger;
        double leftStickY = gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        //create scaling factor for the speed of the robot
        double scale = 1;
        if (gear >= robot.gearTriggerDown) scale = 3.0;

        //variable for the power to each side of the robot
        double leftPower;
        double rightPower;

        //variables for turning and strafing
        double drive;
        double turn;
        double strafePower;

        //now we turn the values from the controller into the
        // robot power through our scale factor
        drive = -leftStickY / scale;
        turn = rightStickX / scale;
        strafePower = leftStickX / scale;

        //This is basic math to decide how the different sides
        // of the robot get power for basic driving
        leftPower = Range.clip(drive + turn, -1, 1);
        rightPower = Range.clip(drive - turn, -1, 1);

        //This decides whether the robot will strafe or
        // drive normally, based of the right trigger (strafeTrigger)
        if (strafeTrigger >= robot.strafeTriggerDown) {
            robot.setStrafePower(strafePower);
        } else {
            robot.setPower(leftPower, rightPower);
        }

    }

    public int goToTarget(Robot2020 robot, int state) {

        // how close is close enough?
        float closeEnoughInches = (float) 6.0;
        float closeEnoughXInches = (float) 6.0;
        float closeEnoughYInches = (float) 6.0;
        float closeEnoughDegrees = (float) 1.0;

        // we are here
        RobotLocation l = robot.getRobotLocation();

        // target is here
        RobotLocation target = new RobotLocation();
        target.x = (float) 0.0;
        target.y = (float) 0.0;
        target.roll = (float) 0.0;

        // state transitions
        switch (state) {
            case GOTO_DONE:
                // if we are close enough, don't do anything
                // otherwise, start the sequence
                if ((l.distance(target) > closeEnoughInches) | (Math.abs(l.roll) > closeEnoughDegrees)) {
                    state = GOTO_ALIGN1;
                }
                break;
            case GOTO_ALIGN1:
                // if we aren't pointed correctly, rotate, otherwise
                // we can move in the x - direction
                // TODO: use the robot gyro instead of vuforia?
                if (Math.abs(l.roll - target.roll) > closeEnoughDegrees) {
                    // rotate to target angle
                } else {
                    // we are close enough, so move in the x direction
                    state = GOTO_MOVEX;
                }
                break;
            case GOTO_MOVEX:
                if (Math.abs(l.x - target.x) > closeEnoughXInches) {
                    if (l.x > target.x) {
                        // move backwards
                    } else {
                        // move forwards
                    }
                } else {
                    // move on to moving in the y direction
                    state = GOTO_MOVEY;
                }
                break;
            case GOTO_MOVEY:
                if (Math.abs(l.y - target.y) > closeEnoughYInches) {
                    if (l.y > target.y) {
                        // move left
                    } else {
                        // move right
                    }
                } else {
                    // move on to last align
                    state = GOTO_ALIGN2;
                }
                break;

            case GOTO_ALIGN2:
                // if we aren't pointed correctly, rotate, otherwise
                // we can move in the x - direction
                // TODO: use the robot gyro instead of vuforia?
                if (Math.abs(l.roll - target.roll) > closeEnoughDegrees) {
                    // rotate to target angle
                } else {
                    // we are close enough, so be done
                    state = GOTO_DONE;
                }
                break;
            default:
                break;

        }
        return state;
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