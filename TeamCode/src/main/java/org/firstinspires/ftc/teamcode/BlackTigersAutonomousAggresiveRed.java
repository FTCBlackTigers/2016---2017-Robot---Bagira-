package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by user on 07/01/2017.
 */

@Disabled
    @Autonomous(name = "Black Tigers Aggro Red Auto ", group = "BlackTigers Auto")
    public class BlackTigersAutonomousAggresiveRed extends LinearVisionOpMode {

    BlackTigersHardware robot = new BlackTigersHardware();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        robot.gyro.calibrate();

        robot.gyro.resetZAxisIntegrator();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForVisionStart();
        this.setCamera(Cameras.SECONDARY);
        this.setFrameSize(new Size(1080, 720));
        enableExtension(Extensions.BEACON);
        enableExtension(Extensions.ROTATION);
        enableExtension(Extensions.CAMERA_CONTROL);
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);
        beacon.setColorToleranceRed(0); //change
        beacon.setColorToleranceBlue(0); //change
        rotation.setIsUsingSecondaryCamera(true);
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();


        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.addData("gyro", robot.gyro.getHeading());
        telemetry.update();


        waitForStart();
        if (runtime.seconds() < 5) {
            robot.shootingMotor.setPower(0.9);
            robot.collectionMotor.setPower(1);
            robot.reloadingMotor.setPower(0.9);
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }
        RobotUtilities.moveForward(0.95, 200, 3, this, robot, telemetry); // Power:1 Distance:55 CM Time:3
        RobotUtilities.gyroRotate(45, robot, telemetry, this);
        RobotUtilities.moveForward(0.95, -18, 1, this, robot, telemetry);

        while (opModeIsActive()) {

            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.update();
//
//            if (beacon.getAnalysis().isLeftBlue()) {
//                robot.beaconsServo.setPosition(0.40);
//            } else if (beacon.getAnalysis().isRightBlue()) {
//                robot.beaconsServo.setPosition(0.66);
//            }// beacon analysis and reaction
            waitOneFullHardwareCycle();
        }

    }
}

