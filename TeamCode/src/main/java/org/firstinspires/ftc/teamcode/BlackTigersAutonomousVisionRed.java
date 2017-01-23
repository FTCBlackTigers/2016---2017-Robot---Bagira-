package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;


/**
 * Created by user on 29/12/2016.
 */
@Autonomous(name = "Black Tigers Vision Red Auto", group = "BlackTigers Auto")
public class BlackTigersAutonomousVisionRed extends LinearVisionOpMode {

    BlackTigersHardware robot = new BlackTigersHardware();

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
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);
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
        RobotUtilities.moveForward(0.95, -45, 3,this,robot,telemetry); // Power:1 Distance:55 CM Time:3
        RobotUtilities.gyroRotate(-45, robot, telemetry, this);
        RobotUtilities.moveForward(0.95, -118, 5,this, robot, telemetry);
        RobotUtilities.gyroRotate(-45, robot, telemetry, this);
       if(beacon.getAnalysis().isLeftRed()){
            robot.beaconsServo.setPosition(0.0);
        }else if(beacon.getAnalysis().isRightRed()){
            robot.beaconsServo.setPosition(0.33);
        }// beacon analysis and reaction
        RobotUtilities.moveForward(0.95, -47, 3,this,robot,telemetry); // Power:1 Distance:20 CM Time:2
       robot.beaconsServo.setPosition(0);
       RobotUtilities.moveForward(0.95, 60, 3,this,robot,telemetry); // Power:1 Distance:20 CM Time:2
       RobotUtilities.gyroRotate(-90,robot,telemetry,this);
       RobotUtilities.moveForward(0.95, 120, 7,this,robot,telemetry);
       RobotUtilities.gyroRotate(90,robot,telemetry,this);
       RobotUtilities.moveForward(0.95 , -20, 3,this,robot,telemetry);
        if(beacon.getAnalysis().isLeftRed()){
            robot.beaconsServo.setPosition(0.0);
        }else if(beacon.getAnalysis().isRightRed()){
            robot.beaconsServo.setPosition(0.33);
        }// beacon analysis and reaction
        RobotUtilities.moveForward(0.95, -40,3,this,robot,telemetry); // Power:1 Distance:20 CM Time:2


        while (opModeIsActive()) {

            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.update();
            waitOneFullHardwareCycle();
    }

}

}