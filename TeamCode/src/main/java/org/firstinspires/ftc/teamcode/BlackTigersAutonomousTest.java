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
public class BlackTigersAutonomousTest extends LinearVisionOpMode {

    BlackTigersHardware robot   = new BlackTigersHardware();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 560 ;
    static final double     DRIVE_GEAR_REDUCTION    = 3.8 ;
    static final double     WHEEL_DIAMETER_CM  = 10.16 ;
    static final double     COUNTS_PER_CM        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);

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
        telemetry.update();


        waitForStart();
        encoderDrive(1, -55, -55, 3); // Power:1 Distance:55 CM Time:3
        encoderDrive(1, 16, -16, 4); // Power:1 Rotation Distance:6.6 Angle: 47.6 Time: 4
        encoderDrive(1, -100 , -100, 7); // Power:1 Distance:215 CM Time:15
        encoderDrive(1, -6, 6, 3);
        encoderDrive(1, -105 ,-105 ,7);
        encoderDrive(1, 30, -30, 5); // Power:1 Distance:55 CM Time:3
        encoderDrive(1, -30, -30, 2); // Power:1 Distance:20 CM Time:2
        if(beacon.getAnalysis().isLeftBlue()){
            robot.beaconsServo.setPosition(0.40);
        }else if(beacon.getAnalysis().isRightBlue()){
            robot.beaconsServo.setPosition(0.66);
        }// beacon analysis and reaction
        encoderDrive(1, -47, -47, 2); // Power:1 Distance:20 CM Time:2
        Thread.sleep(1000);
        encoderDrive(1,47,47,2);//after beacon press return to last position
        encoderDrive(1, 34, -34, 3); // Power:1 Distance:10.6 CM Time:3
        encoderDrive(1, -137, -137, 7); // Power:1 Distance:120 CM Time:7
        encoderDrive(1, -40, 40, 3); // Power:1 Distance:10.6 CM Time:3
        encoderDrive(1, -20 , -20 , 4);

        robot.reloadingMotor.setPower(1);
        robot.shootingMotor.setPower(1);

        if(beacon.getAnalysis().isLeftBlue()){
            robot.beaconsServo.setPosition(0.34);
        }else if(beacon.getAnalysis().isRightBlue()){
            robot.beaconsServo.setPosition(0.66);
        }// beacon analysis and reaction
        encoderDrive(1, -42, -42, 2); // Power:1 Distance:20 CM Time:2

        while (opModeIsActive()) {

            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.update();

            waitOneFullHardwareCycle();
        }
    }

    public void encoderDrive(double speed,
                             double leftCM, double rightCM,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftCM * COUNTS_PER_CM);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightCM * COUNTS_PER_CM);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
                    robot.leftMotor.setPower(Math.abs(speed));
                    robot.rightMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())){

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.addData("Power", "left: %f, right: %f", robot.leftMotor.getPower(), robot.rightMotor.getPower());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }
}