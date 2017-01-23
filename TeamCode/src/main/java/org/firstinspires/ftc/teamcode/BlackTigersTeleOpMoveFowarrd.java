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


@TeleOp(name = "Black Tiger TeleopFowarrd", group = "BlackTigers")
// @Autonomous(...) is the other common choice
public class BlackTigersTeleOpMoveFowarrd extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftMotor   = null;
    public DcMotor  rightMotor  = null;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftMotor   = hardwareMap.dcMotor.get("left_drive");
        rightMotor  = hardwareMap.dcMotor.get("right_drive");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
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
        leftPower = RobotUtilities.normalizePower(leftPower);
        rightPower = RobotUtilities.normalizePower(rightPower);
        if(gamepad1.right_bumper){
            leftPower=leftPower/4;
            rightPower=rightPower/4;
        }
        DbgLog.msg("Left Stick: " + leftPower + "; Right Stick: " + rightPower);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);}
//        if (gamepad2.right_trigger > 0) {
//            if (runtime.milliseconds() % 40 < 5) {
//                robot.shootingMotor.setPower(Range.clip(robot.shootingMotor.getPower() + 0.02, 0, 0.95));
//            }
//        } else {
//            if (runtime.milliseconds() % 40 < 2) {
//                robot.shootingMotor.setPower(Range.clip(robot.shootingMotor.getPower() / 1.15, 0, 0.95));
//            }
//            if (robot.shootingMotor.getPower() < 0.1) {
//                robot.shootingMotor.setPower(0);
//            }
//        }



    public void stop(){

    }
}



