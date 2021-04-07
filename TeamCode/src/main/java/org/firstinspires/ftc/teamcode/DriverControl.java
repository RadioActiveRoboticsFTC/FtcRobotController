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

    // we currently aren't using this; the idea was for automated navigation
    // with the images
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

        // we aren't using this right now;
        // for the image navigation
        int goToState = GOTO_DONE;

        // this is so that the motor will have already started up when we start the match so that
        // in the match we do not have to wait for it to start up
        robot.shooterMotor.setPower(0.0);

        // TBF: should we do this?
//        robot.releaseServo.setPosition(0);

        // variables for catching button transition for initing IMU
        boolean curIMUBtnState = false;
        boolean prevIMUBtnState = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            double pos = robot.leftDrive.getCurrentPosition();
            //telemetry.addData("pos", pos);
//            robotMotion(robot);

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

            // this handles everything except the motion, things like:
            // shooting, intake motors, etc.
            robotActions(robot);

            // initialize IMU?
            curIMUBtnState = gamepad2.x;
            // init IMU just once for every time x btn is pressed
            if ((curIMUBtnState) && (curIMUBtnState != prevIMUBtnState)) {
                telemetry.addData("Initing IMU ...", "");
                robot.initIMU();
            }
            prevIMUBtnState = curIMUBtnState;



            //countRings(robot);
//            robot.shooterMotor.setPower(gamepad2.right_stick_y*0.5);
//            double shootPower = gamepad2.right_stick_y*0.5;
            idle();

//             Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("shoot power", shootPower);
//
//            telemetry.update();
        }
    }

    // we aren't currently aren't using this, but this uses the images
    // for automatic navigation
    public void robotVision(Robot2020 robot) {

        OpenGLMatrix lastLocation = null;
        float mmPerInch        = 25.4f;

        // check all the trackable targets to see which one (if any) is visible.
        boolean targetVisible = false;
        for (VuforiaTrackable trackable : robot.allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
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

            float halfField = 72f; // inches
            float quadField = (float) (halfField/2.0);

            float xTarget = halfField;
            float yTarget = -quadField;
            float xd = xTarget-x;
            float yd = yTarget-y;
            double distance = Math.sqrt((xd*xd) + (yd*yd));
            telemetry.addData("distance",distance);
            float xy = yd / xd;

            double angle1 = Math.tanh(xy);
            double angle = 90.0 - angle1;

            telemetry.addData("turn to: ", angle);


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
//            robot.setPower(power);
            double currentAngle = robot.getYAxisAngle();

            telemetry.addData("currentAngle:", currentAngle);


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
            robot.setStrafePower(-strafePower);
        } else {
            robot.setPower(leftPower, rightPower);
        }


        // automatic aiminng!
        double targetAngle = 5.0;
        if (gamepad1.right_bumper) {
            telemetry.addLine("Auto Aiming!");
            telemetry.update();
            double curAngle = robot.getYAxisAngle();
            if (curAngle > targetAngle)
                spinRightP(robot, targetAngle, 0.25);
            else
                spinLeftP(robot, targetAngle, 0.25);
        }

//        telemetry.addData("LeftStickX", leftStickX);
//        telemetry.addData("LeftStickY", leftStickY);
//        telemetry.addData("rightStickY", rightStickX);
//        telemetry.addData("LeftPower:", leftPower);
//        telemetry.addData("RightPower:", rightPower);
        double currentAngle = robot.getYAxisAngle();

        telemetry.addData("currentAngle:", currentAngle);
        telemetry.update();
    }

    public void robotActions(Robot2020 robot) {

        // the a button turns on the shooting motor, if pressed
        double shootPower = 0.0;
        if (gamepad2.a)
            shootPower = 0.5;
        robot.shooterMotor.setPower(shootPower);

        // TODO: clean up old commented out code
        if (gamepad2.right_bumper) {
            // shoot the rings!
            robot.raiseRingWall();
            sleep(500);
            robot.pushRingPusher();
        } else {
            // put shooter stuff back in oringinal position
            robot.lowerRingWall();
            sleep(500);
            robot.retractRingPusher();
        }

        // obtaining rings!
        // turns on the intake motors
        if(gamepad2.left_bumper){
            robot.lowerIntakeMotor.setPower(1);
            robot.upperIntakeMotor.setPower(1);
        }
        else{
            robot.lowerIntakeMotor.setPower(0);
            robot.upperIntakeMotor.setPower(0);
        }

        // reverse intake motors to prevent catastrophic event with rings.
        // so we can spit out rings that are stuck.
        if(gamepad2.y){
            robot.lowerIntakeMotor.setPower(-1);
            robot.upperIntakeMotor.setPower(-1);
        }

        // this lefts us aim our shooter with the up/down motion
        double shooterAnglePower = 0.3 * gamepad2.left_stick_y;
        robot.shooterAngleMotor.setPower(shooterAnglePower);

        if (gamepad1.y) {
            robot.wobbleServo.setPosition(0.25);
        } else {
            robot.wobbleServo.setPosition(0.50);
        }

        // button b lowers the intake
        if (gamepad2.b) {
            // raise release arm to lower the intake
            robot.raiseReleaseLatch();
        }
        else{
            // push release arm down
            robot.lowerReleaseLatch();
        }

    }

    // TODO: comment for this function?
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

    // uses the web cam to count rings; returns the number of rings it sees
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

    // TODO: could we share this code from BaseAutonomous, rather then copy it?

    // This function is just like spinLeft, but sets the power proportional to the distance
    // from our target angle; this way we don't have to brake at the end.
    public void spinLeftP(Robot2020 robot, double toAngle, double power) {

        double yAxisAngle;
        yAxisAngle = robot.getYAxisAngle();

        // how far are we trying to go?
        double totalAngularDistance = toAngle-yAxisAngle;
        boolean turning = true;
        double prevAngle = robot.getYAxisAngle();

        // Loop and update the dashboard
        while (opModeIsActive() && turning) {
            //what angle are we at right now?
            yAxisAngle = robot.getYAxisAngle();

            // watch for the 180/-180 border!
            // Recall that forward is 0 degrees, with positive to the left,
            // negative to the right
            if (yAxisAngle < 0 && prevAngle > 0) turning = false;

            prevAngle = yAxisAngle;

            // proportional power
            double percentAngularDistance = (toAngle-yAxisAngle)/totalAngularDistance;
            double proportionalPower = percentAngularDistance * power;
            // make sure we never get down to a power of zero, or we
            // might never finish our turn
            proportionalPower = Range.clip(proportionalPower, .1, power);

            // have we spun far enough yet?
            if (yAxisAngle >= toAngle) turning = false;

            telemetry.addData("yAxis", yAxisAngle);
            telemetry.addData("turning:", turning);
            telemetry.addData("pPower", proportionalPower);
            telemetry.update();

            if (turning) {
                // spin left
                robot.setPower(-proportionalPower, proportionalPower);
            }
        }
        robot.setPower(0);
    }

    // This function is just like spinRight, but sets the power proportional to the distance
    // from our target angle; this way we don't have to brake at the end.
    // TBF: can we combine this with spinLeftP to make one function?
    public void spinRightP (Robot2020 robot, double toAngle, double power) {

        // this code turns right
        double yAxisAngle;
        yAxisAngle = robot.getYAxisAngle();
        boolean turning = true;
        double totalAngularDistance = toAngle-yAxisAngle;

        double prevAngle = robot.getYAxisAngle();

        // Loop and update the dashboard
        while (opModeIsActive() && turning) {
            // what angle are we at right now?
            yAxisAngle = robot.getYAxisAngle();

            // watch for the border at -180/180
            if (yAxisAngle > 0 && prevAngle < 0) turning = false;

            prevAngle = yAxisAngle;

            // proportional power
            double percentAngularDistance = (toAngle-yAxisAngle)/totalAngularDistance;
            double proportionalPower = percentAngularDistance * power;
            // make sure we never go to a power of zero and never finish our turning
            proportionalPower = Range.clip(proportionalPower, .1, power);

            // have we spun enough yet?
            if (yAxisAngle <= toAngle) turning = false;

            telemetry.addData("yAxis", yAxisAngle);
            telemetry.addData("pPower", proportionalPower);
            telemetry.addData("turning:", turning);
            telemetry.update();

            if (turning) {
                // spin right
                robot.setPower(proportionalPower, -proportionalPower);
            }
        }

        robot.setPower(0);
    }

}