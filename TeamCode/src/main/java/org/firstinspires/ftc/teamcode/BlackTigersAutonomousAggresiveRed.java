package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
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

        int heading;
        static final double COUNTS_PER_MOTOR_REV = 560;
        static final double DRIVE_GEAR_REDUCTION = 3.8;
        static final double ROTATE_SPEED = 0.25;
        static final double WHEEL_DIAMETER_CM = 10.16;
        static final double CORRECTION_FACOTR = 0.15;
        static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_CM * 3.1415);


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
            if(runtime.seconds()<5) {
                robot.shootingMotor.setPower(0.9);
                robot.collectionMotor.setPower(1);
                robot.reloadingMotor.setPower(0.9);
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
            }
            encoderDrive(0.95, 200, 200, 3); // Power:1 Distance:55 CM Time:3
            gyroRotate(45);
            encoderDrive(0.95, -18, -18,1);

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

                int startDirection = robot.gyro.getHeading();

                // keep looping while we are still active, and there is time left, and both motors are running.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())){
                    int direction = robot.gyro.getHeading() - startDirection;
                    if(direction > 0) {
                        if(direction <= 180) {
                            robot.leftMotor.setPower(Range.clip(Math.abs(speed) - getErrorFraction(direction), 0,1));
                        } else {
                            direction = Math.abs(direction -360);
                            robot.rightMotor.setPower(Range.clip(Math.abs(speed) - getErrorFraction(direction), 0, 1));
                        }
                    } else {
                        robot.leftMotor.setPower(Math.abs(speed));
                        robot.rightMotor.setPower(Math.abs(speed));
                    }
                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.leftMotor.getCurrentPosition(),
                            robot.rightMotor.getCurrentPosition());
                    telemetry.addData("Power", "left: %f, right: %f", robot.leftMotor.getPower(), robot.rightMotor.getPower());
                    telemetry.update();
                }

                // Stop all motion;
                int i = 0;
                while(opModeIsActive() && i < 4 && runtime.milliseconds() % 20 == 0) {
                    robot.leftMotor.setPower(Range.clip(robot.leftMotor.getPower()/2, 0, 1));
                    robot.rightMotor.setPower(Range.clip(robot.rightMotor.getPower()/2, 0, 1));
                    i++;
                }

                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }

        }

        public double getErrorFraction(int direction) {
            return Range.clip(direction * CORRECTION_FACOTR, 0, 1);
        }

        public void gyroRotate(int degrees) {
            degrees=-degrees;
            int startPosition = robot.gyro.getHeading();
            int targetPosition = startPosition+degrees;
            while(opModeIsActive() && Math.abs(Math.abs(degrees)-Math.abs(robot.gyro.getHeading()-startPosition)) > 3) {
                if(degrees < 0) {
                    robot.leftMotor.setPower(ROTATE_SPEED);
                    robot.rightMotor.setPower(-ROTATE_SPEED);
                } else {
                    robot.leftMotor.setPower(-ROTATE_SPEED);
                    robot.rightMotor.setPower(ROTATE_SPEED);
                }
                telemetry.addData("rotate0", "Starting Turn at %d", startPosition);
                telemetry.addData("rotate1", "Finishing Turn at %d", targetPosition);
                telemetry.addData("rotate2", "Robot gyro currently at %d", robot.gyro.getHeading());
                telemetry.update();
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }

    }

