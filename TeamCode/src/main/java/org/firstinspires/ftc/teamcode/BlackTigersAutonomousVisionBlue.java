package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;



/**
 * Created by user on 29/12/2016.
 */
@Autonomous(name = "Black Tigers Vision Blue Auto", group = "BlackTigers Auto")
public class BlackTigersAutonomousVisionBlue extends LinearVisionOpMode {

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

        RobotUtilities.moveForward(0.95, -45, 3, this, robot, telemetry); // Power:1 Distance:55 CM Time:3
        RobotUtilities.gyroRotate(45, robot, telemetry, this);
        RobotUtilities.moveForward(0.95, -118, 5, this, robot, telemetry);
        RobotUtilities.gyroRotate(45, robot, telemetry, this);
        if (beacon.getAnalysis().isLeftBlue()) {
            robot.beaconsServo.setPosition(0.0);
        } else if (beacon.getAnalysis().isRightBlue()) {
            robot.beaconsServo.setPosition(0.33);
        }// beacon analysis and reaction
        RobotUtilities.moveForward(0.95, -55,3, this, robot, telemetry); // Power:1 Distance:20 CM Time:2
        robot.beaconsServo.setPosition(0);

        RobotUtilities.moveForward(0.95, 60, 3, this, robot, telemetry); // Power:1 Distance:20 CM Time:2
        RobotUtilities.gyroRotate(90, robot, telemetry, this);
        RobotUtilities.moveForward(0.95, 120, 7, this, robot, telemetry);
        RobotUtilities.gyroRotate(-90, robot, telemetry, this);
        RobotUtilities.moveForward(0.95, -20, 3, this, robot, telemetry);
        if (beacon.getAnalysis().isLeftBlue()) {
            robot.beaconsServo.setPosition(0.0);
        } else if (beacon.getAnalysis().isRightBlue()) {
            robot.beaconsServo.setPosition(0.33);
        }// beacon analysis and reaction
        RobotUtilities.moveForward(0.95, -40,3, this, robot, telemetry); // Power:1 Distance:20 CM Time:2


//        moveForward(0.75, 40, -40, 4); // Power:1 Rotation Distance:6.6 Angle: 47.6 Time: 4
//        moveForward(1, -86, -86, 7); // Power:1 Distance:100 CM Time:15
//        moveForward(0.75,27, -27, 5); // Power:1 Distance:55 CM Time:3
//        moveForward(1, 10, 10, 7); // Power:1 Distance:100 CM Time:15
//        moveForward(1, -47, -47, 2); // Power:1 Distance:20 CM Time:2
//        Thread.sleep(1000);\
//        moveForward(1,47,47,2);//after beacon press return to last position
//        moveForward(0.75, 34, -34, 3); // Power:1 Distance:10.6 CM Time:3
//        moveForward(1, -139, -139, 7); // Power:1 Distance:120 CM Time:7
//        moveForward(0.75, -40, 40, 3); // Power:1 Distance:10.6 CM Time:3
//        moveForward(1, -20 , -20 , 4);
//
//        robot.reloadingMotor.setPower(1);
//        robot.shootingMotor.setPower(1);
//
//        if(beacon.getAnalysis().isLeftBlue()){
//            robot.beaconsServo.setPosition(0.34);
//        }else if(beacon.getAnalysis().isRightBlue()){
//            robot.beaconsServo.setPosition(0.66);
//        }// beacon analysis and reaction
//        moveForward(1, -42, -42, 2); // Power:1 Distance:20 CM Time:2


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