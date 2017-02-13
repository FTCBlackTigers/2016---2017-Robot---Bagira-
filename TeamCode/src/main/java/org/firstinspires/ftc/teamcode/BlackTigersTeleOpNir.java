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
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Black Tiger Teleop Nir", group = "BlackTigers")
// @Autonomous(...) is the other common choice
public class BlackTigersTeleOpNir extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    BlackTigersHardware robot = new BlackTigersHardware();
    boolean isReloading = false;
    boolean isCollecting = false;
    boolean isShootingFinishedSpeeding = false;
    boolean isShootingFinishedSlowing = true;
    final double ReloadingSpeed = -0.85;
    final double CollectionSpeed = 1;
    final double normalSpeed = 0.75;
    final double maxSpeed = 0.90;
    final double minSpeed = 0.60;


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


        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

//        DbgLog.msg("Left Stick: " + leftPower + "; Right Stick: " + rightPower);
        if ((!gamepad1.right_bumper && !gamepad1.left_bumper) || (gamepad1.left_bumper && gamepad1.right_bumper)) {
            robot.leftMotor.setPower(RobotUtilities.normalizePower(leftPower));
            robot.rightMotor.setPower(RobotUtilities.normalizePower(rightPower));
            telemetry.addData("Motors: ", "Left Stick: " + robot.leftMotor.getPower() + "; Right Stick: " + robot.rightMotor.getPower());
        } else if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            robot.leftMotor.setPower(RobotUtilities.maxNormalizePower(leftPower));
            robot.rightMotor.setPower(RobotUtilities.maxNormalizePower(rightPower));
            telemetry.addData("Motors: ", "Left Stick: " + robot.leftMotor.getPower() + "; Right Stick: " + robot.rightMotor.getPower());
        } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            robot.leftMotor.setPower(RobotUtilities.normalizePower(leftPower) * 2 / 3);
            robot.rightMotor.setPower(RobotUtilities.normalizePower(rightPower) * 2 / 3);
            telemetry.addData("Motors: ", "Left Stick: " + robot.leftMotor.getPower() + "; Right Stick: " + robot.rightMotor.getPower());
        }


        if (gamepad2.right_trigger > 0) {
            robot.shootingMotor.setPower(1);
        } else {
            robot.shootingMotor.setPower(0);
        }
//       shooting motor need to be added
        if (gamepad2.left_bumper && gamepad2.a && gamepad2.right_bumper && gamepad2.x && gamepad2.b) {
            robot.reloadingMotor.setPower(0);
            isCollecting = false;
        } else if (gamepad2.a && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.x && !gamepad2.b && !gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.collectionMotor.setPower(CollectionSpeed);
            robot.reloadingMotor.setPower(ReloadingSpeed);
        } else if (!gamepad2.a && gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.x && !gamepad2.b && !gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.collectionMotor.setPower(-CollectionSpeed);
            robot.reloadingMotor.setPower(-ReloadingSpeed);
        } else if (!gamepad2.a && !gamepad2.left_bumper && gamepad2.right_bumper && !gamepad2.x && !gamepad2.b && !gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.collectionMotor.setPower(CollectionSpeed);
            robot.reloadingMotor.setPower(0);
        } else if (!gamepad2.a && !gamepad2.left_bumper && !gamepad2.right_bumper && gamepad2.x && !gamepad2.b && !gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.collectionMotor.setPower(0);
            robot.reloadingMotor.setPower(ReloadingSpeed);
            isCollecting = false;
        } else if (!gamepad2.a && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.x && gamepad2.b && !gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.reloadingMotor.setPower(-ReloadingSpeed / 2);
            robot.collectionMotor.setPower(0);
        }
//         else  if (!gamepad2.a && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.x && !gamepad2.b && !gamepad2.dpad_up && gamepad2.dpad_down){
//            robot.ballMotorRight.setPower(-0.80);
//            robot.ballMotorLeft.setPower(-0.80);
//        }else  if (!gamepad2.a && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.x && !gamepad2.b && gamepad2.dpad_up && !gamepad2.dpad_down){
//            robot.ballMotorRight.setPower(0.80);
//            robot.ballMotorLeft.setPower(0.80);
        else {
            robot.collectionMotor.setPower(0);
            robot.reloadingMotor.setPower(0);
//            robot.ballMotorRight.setPower(0);
//            robot.ballMotorLeft.setPower(0);
        }

        }


        @Override
        public void stop () {
        }

    }
