package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;
/*
 * Created by user on 01/02/2017.
 */


@Autonomous(name = "BlackTigersAutoRedShoot&Parking", group = "BlackTigers Auto")
    public class BlackTigersAutoRedShootAndParking extends LinearVisionOpMode {

    BlackTigersHardware robot = new BlackTigersHardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForVisionStart();

        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();
        robot.shootingMotor.setPower(1);
        sleep(1500);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed, 35,10, this, robot, telemetry);
        robot.reloadingMotor.setPower(-0.95);
        sleep(800);
        robot.reloadingMotor.setPower(0);
        sleep(4000);
        robot.reloadingMotor.setPower(-0.95);
        sleep(1300);
        robot.shootingMotor.setPower(0);
        robot.reloadingMotor.setPower(0);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed, 35,10, this, robot, telemetry);
        RobotUtilities.gyroRotate(60, robot, telemetry, this);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed, -95,10, this, robot, telemetry);


        while (opModeIsActive()) {
            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.update();
            waitOneFullHardwareCycle();
        }
    }
}