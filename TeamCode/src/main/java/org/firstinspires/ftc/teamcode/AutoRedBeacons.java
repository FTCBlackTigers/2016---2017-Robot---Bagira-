package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;
/*
 * Created by user on 01/02/2017.
 */


@Autonomous(name = "Red-Beacons", group = "BlackTigers Auto")
    public class AutoRedBeacons extends LinearVisionOpMode {

    BlackTigersHardware robot = new BlackTigersHardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RobotUtilities.calibrategyro(telemetry, robot, this);
        enableExtension(Extensions.BEACON);
        enableExtension(Extensions.ROTATION);
        enableExtension(Extensions.CAMERA_CONTROL);
        RobotUtilities.cameraSetup(this);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForVisionStart();

        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();

        RobotUtilities.moveForward(RobotUtilities.normalSpeed, 52, 10, this, robot, telemetry);
        RobotUtilities.gyroRotate(-37, robot, telemetry, this);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed * 0.5, 90, 10, this, robot, telemetry);
        sleep(200);
        RobotUtilities.gyroRotate(37, robot, telemetry, this);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed, 10, 10, this, robot, telemetry);
        sleep(500);
        while (!beacon.getAnalysis().isRightKnown() || !beacon.getAnalysis().isLeftKnown()) {}
        boolean isRedRight = beacon.getAnalysis().isRightRed();
        sleep(200);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed, -4, 10, this, robot, telemetry);
        sleep(500);
        RobotUtilities.gyroRotate(84, robot, telemetry, this);
        robot.shootingMotor.setPower(0.75);
        sleep(200);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed / 3, -70, 10, this, robot, telemetry);
        RobotUtilities.moveForward(0.35, 37, 10, this, robot, telemetry);
        sleep(1500);
        robot.reloadingMotor.setPower(-0.75);
        sleep(2500);
        robot.shootingMotor.setPower(0);
        robot.reloadingMotor.setPower(0);
        if (!isRedRight) {
            RobotUtilities.moveForward(RobotUtilities.normalSpeed / 2, -58, 10, this, robot, telemetry);
            sleep(700);
            RobotUtilities.moveForward(RobotUtilities.normalSpeed / 2, 35, 10, this, robot, telemetry);}
        sleep(0500);
        RobotUtilities.gyroRotate(-89, robot, telemetry, this);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed / 1.5, 120, 10, this, robot, telemetry);
        sleep(500);
        isRedRight = beacon.getAnalysis().isRightRed();
        sleep(200);
        if (isRedRight) {
            RobotUtilities.gyroRotate(85, robot, telemetry, this);
            sleep(200);
            RobotUtilities.moveForward(RobotUtilities.normalSpeed / 2, -60, 10, this, robot, telemetry);
            RobotUtilities.moveForward(RobotUtilities.normalSpeed / 2, 15, 10, this, robot, telemetry);
        }else {
        RobotUtilities.moveForward(RobotUtilities.normalSpeed / 2, -17, 10, this, robot, telemetry);
            RobotUtilities.gyroRotate(85, robot, telemetry, this);
            sleep(200);
            RobotUtilities.moveForward(RobotUtilities.normalSpeed / 2, -60, 10, this, robot, telemetry);
            RobotUtilities.moveForward(RobotUtilities.normalSpeed / 2, 15, 10, this, robot, telemetry);}

        while (opModeIsActive()) {
            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.update();
            waitOneFullHardwareCycle();
        }
    }
}