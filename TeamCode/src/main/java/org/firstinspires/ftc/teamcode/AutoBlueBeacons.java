package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
/*
 * Created by user on 01/02/2017.
 */


@Autonomous(name = "Blue-Beacons", group = "BlackTigers Auto")
public class AutoBlueBeacons extends LinearVisionOpMode {

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
        // set up camera for beacon detection
        enableExtension(VisionOpMode.Extensions.BEACON);
        enableExtension(VisionOpMode.Extensions.ROTATION);
        enableExtension(VisionOpMode.Extensions.CAMERA_CONTROL);
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

        // Drive and Turn toward the first beacon
        RobotUtilities.moveForward(RobotUtilities.normalSpeed, -55, 10, this, robot, telemetry);
        RobotUtilities.gyroRotate(41, robot, telemetry, this);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed, -103, 10, this, robot, telemetry);
        sleep(200);
        // align the camera with beacon
        RobotUtilities.gyroRotate(-31, robot, telemetry, this);
        sleep(100);
        // Read what color is the beacon
        while (!beacon.getAnalysis().isRightKnown() || !beacon.getAnalysis().isLeftKnown()) {

        }
        RobotUtilities.moveForward(RobotUtilities.normalSpeed / 3, 6, 10, this, robot, telemetry);
        boolean isBlueRight = beacon.getAnalysis().isRightBlue();
        telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
        telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
        telemetry.update();
        // move the robot to a position to press the beacon
        sleep(200);
        RobotUtilities.gyroRotate(86  , robot, telemetry, this);
        // warm up the shooting mechanism for shooting
        robot.shootingMotor.setPower(0.75);
        sleep(200);
        // press the beacon, and move forward to shooting position
        RobotUtilities.moveForward(RobotUtilities.normalSpeed / 2, -63, 10, this, robot, telemetry);
        RobotUtilities.moveForward(0.35, 35, 10, this, robot, telemetry);
        sleep(1500);
        // activate the shooting mechanism
        robot.reloadingMotor.setPower(-0.75);
        sleep(2500);
        robot.shootingMotor.setPower(0);
        robot.reloadingMotor.setPower(0);
        // if the beacon is not in the right color, press it again
        if (!isBlueRight) {
            RobotUtilities.moveForward(RobotUtilities.normalSpeed / 2, -53, 10, this, robot, telemetry);
            sleep(700);
            RobotUtilities.moveForward(RobotUtilities.normalSpeed, 35, 10, this, robot, telemetry);
        }
        //Driving towards the 2nd Beacon
        sleep(500);
        RobotUtilities.gyroRotate(-85, robot, telemetry, this);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed, -114, 10, this, robot, telemetry);
        sleep(500);
        //Read what color is the Beacon
        isBlueRight = beacon.getAnalysis().isRightBlue();
        telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
        telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
        // if the beacon is not in the right color, press it again
        if (!isBlueRight) {
            RobotUtilities.moveForward(RobotUtilities.normalSpeed, -15, 5, this, robot, telemetry);
        }
        RobotUtilities.gyroRotate(85, robot, telemetry, this);
        sleep(200);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed / 2, -66, 10, this, robot, telemetry);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed / 3, 5, 10, this, robot, telemetry);




        while (opModeIsActive()) {
            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            waitOneFullHardwareCycle();
        }
    }
}