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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Working Remote Control", group="Linear Opmode")
public class DriverControl extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();


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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double pos = robot.leftDrive.getCurrentPosition();
            telemetry.addData("pos", pos);

            // moves the robot
            robotMotion(robot);
            idle();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
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
}