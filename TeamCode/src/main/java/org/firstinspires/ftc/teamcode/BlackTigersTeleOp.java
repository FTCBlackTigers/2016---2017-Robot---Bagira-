/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Black Tiger Teleop", group = "BlackTigers")
// @Autonomous(...) is the other common choice
public class BlackTigersTeleOp extends OpMode {
    //    we define basic configuration
    private ElapsedTime runtime = new ElapsedTime();
    BlackTigersHardware robot = new BlackTigersHardware();
    boolean isCollecting = false;
    final double ReloadingSpeed = -0.75;
    final double CollectionSpeed = 1;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        //GAMEPAD 1
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;
        DbgLog.msg("Left Stick: " + leftPower + "; Right Stick: " + rightPower);
        // we give our driver the option to move fast slow in the normal speed and control our robot moving
        // buy clicking on a button
        if ((!gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.dpad_up && !gamepad1.dpad_down) || (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.dpad_up && gamepad1.dpad_down)) {
            robot.leftMotor.setPower(RobotUtilities.normalizePower(leftPower));
            robot.rightMotor.setPower(RobotUtilities.normalizePower(rightPower));
            telemetry.addData("Motors: ", "Left Stick: " + robot.leftMotor.getPower() + "; Right Stick: " + robot.rightMotor.getPower());
        } else if (!gamepad1.left_bumper && gamepad1.right_bumper && !gamepad1.dpad_up && !gamepad1.dpad_down) {
            robot.leftMotor.setPower(RobotUtilities.maxNormalizePower(leftPower));
            robot.rightMotor.setPower(RobotUtilities.maxNormalizePower(rightPower));
            telemetry.addData("Motors: ", "Left Stick: " + robot.leftMotor.getPower() + "; Right Stick: " + robot.rightMotor.getPower());
        } else if (!gamepad1.right_bumper && gamepad1.left_bumper && !gamepad1.dpad_up && !gamepad1.dpad_down) {
            robot.leftMotor.setPower(RobotUtilities.normalizePower(leftPower) / 5);
            robot.rightMotor.setPower(RobotUtilities.normalizePower(rightPower) / 5);
            telemetry.addData("Motors: ", "Left Stick: " + robot.leftMotor.getPower() + "; Right Stick: " + robot.rightMotor.getPower());
        } else if (!gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.dpad_up && !gamepad1.dpad_down) {
            robot.leftMotor.setPower(0.75);
            robot.rightMotor.setPower(0.75);
            telemetry.addData("Motors: ", "Left Stick: " + robot.leftMotor.getPower() + "; Right Stick: " + robot.rightMotor.getPower());
        } else if (!gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.dpad_up && gamepad1.dpad_down) {
            robot.leftMotor.setPower(-0.75);
            robot.rightMotor.setPower(-0.75);
            telemetry.addData("Motors: ", "Left Stick: " + robot.leftMotor.getPower() + "; Right Stick: " + robot.rightMotor.getPower());
        }


        //GAMEPAD 2
        // The operator can activate the shooting motor, The collecting motor, The cap ball motors
        // and the reloading motor.
        if (gamepad2.right_trigger > 0) {
            robot.shootingMotor.setPower(1);
        } else {
            robot.shootingMotor.setPower(0);
        }
        if (gamepad2.left_trigger > 0) {
            robot.ballMotorLeft.setPower(-1);
            robot.ballMotorRight.setPower(-1);
        } else if (!gamepad2.dpad_down && !gamepad2.dpad_up) {
            robot.ballMotorLeft.setPower(0);
            robot.ballMotorRight.setPower(0);
        }


        if (gamepad2.left_bumper && gamepad2.a && gamepad2.right_bumper && gamepad2.x && gamepad2.b && !gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.reloadingMotor.setPower(0);
            isCollecting = false;
        } else if (gamepad2.a && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.x && !gamepad2.b && !gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.collectionMotor.setPower(CollectionSpeed);
            robot.reloadingMotor.setPower(ReloadingSpeed);
        } else if (!gamepad2.a && gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.x && !gamepad2.b && !gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.collectionMotor.setPower(-CollectionSpeed);
            robot.reloadingMotor.setPower(-ReloadingSpeed);
        } else if (!gamepad2.a && !gamepad2.left_bumper && gamepad2.right_bumper && !gamepad2.x && !gamepad2.b && !gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.collectionMotor.setPower(CollectionSpeed);
            robot.reloadingMotor.setPower(0);
        } else if (!gamepad2.a && !gamepad2.left_bumper && !gamepad2.right_bumper && gamepad2.x && !gamepad2.b && !gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.collectionMotor.setPower(0);
            robot.reloadingMotor.setPower(ReloadingSpeed);
            isCollecting = false;
        } else if (!gamepad2.a && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.x && gamepad2.b && !gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.reloadingMotor.setPower(-ReloadingSpeed / 2);
            robot.collectionMotor.setPower(0);
        } else if (!gamepad2.a && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.x && !gamepad2.b && !gamepad2.dpad_up && gamepad2.dpad_down && !gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.ballMotorRight.setPower(0.70);
            robot.ballMotorLeft.setPower(0.70);
        } else if (!gamepad2.a && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.x && !gamepad2.b && gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.ballMotorRight.setPower(-1);
            robot.ballMotorLeft.setPower(-1);
        } else {
            robot.collectionMotor.setPower(0);
            robot.reloadingMotor.setPower(0);
            if (gamepad2.left_trigger == 0) {
                robot.ballMotorRight.setPower(0);
                robot.ballMotorLeft.setPower(0);
            }
        }


        telemetry.addData("shooting speed", "%f", robot.shootingMotor.getPower());
        telemetry.addData("Path2", "Running at %7d :%7d", robot.leftMotor.getCurrentPosition()
                , robot.rightMotor.getCurrentPosition());
        telemetry.addData("gyro", robot.gyro.getHeading());
        telemetry.update();


    }

    @Override
    public void stop() {
    }

}
