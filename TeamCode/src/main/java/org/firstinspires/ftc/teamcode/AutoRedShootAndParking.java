package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;
/*
 * Created by user on 01/02/2017.
 */


@Autonomous(name = "Red-Shoot&Parking", group = "BlackTigers Auto")
    public class AutoRedShootAndParking extends LinearVisionOpMode {

    BlackTigersHardware robot = new BlackTigersHardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize hardware
        robot.init(hardwareMap);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

// calibrate the gyro
        RobotUtilities.calibrategyro(telemetry, robot, this);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForVisionStart();

        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();
        //Starting the shooting mechanism
        robot.shootingMotor.setPower(1);
        sleep(1500);
        //Driving towards the Center Of Vortex
        RobotUtilities.moveForward(RobotUtilities.normalSpeed/2, 30,10, this, robot, telemetry);
        //Shooting 2 Balls
        robot.reloadingMotor.setPower(-0.95);
        sleep(800);
        robot.reloadingMotor.setPower(0);
        sleep(4000);
        robot.reloadingMotor.setPower(-0.95);
        sleep(1300);
        robot.shootingMotor.setPower(0);
        robot.reloadingMotor.setPower(0);
        //Driving towards the Corner Of Vortex
        RobotUtilities.moveForward(RobotUtilities.normalSpeed, 35,10, this, robot, telemetry);
        RobotUtilities.gyroRotate(60, robot, telemetry, this);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed/2, -95,10, this, robot, telemetry);
//the robot is parked on the Corner Of Vortex

        while (opModeIsActive()) {
            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.update();
            waitOneFullHardwareCycle();
        }
    }
}