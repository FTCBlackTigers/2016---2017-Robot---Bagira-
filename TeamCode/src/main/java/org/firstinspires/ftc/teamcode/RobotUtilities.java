package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.BeaconExtension;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by user on 05/11/2016.
 */

public class RobotUtilities {

    private static ElapsedTime runtime = new ElapsedTime();
    //Here we define the various constants for our robot
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_CM = 10.16;
    static final double CORRECTION_FACOTR = 0.08;
    final static double normalSpeed = 0.95;
    static final double ROTATE_SPEED = 0.7;

    // Here we convert encoder ticks to centimeters.
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);

    //We use this method in our teleop program to help our driver to control the driving better
    public static double normalizePower(double power) {
        return normalSpeed * 0.95 * Range.clip(Math.pow(power, 7), -1, 1);
    }

    public static double maxNormalizePower(double power) {
        return 0.45 * Range.clip(Math.pow(power, 7), -1, 1);
    }

    //We use this method in the Autonomous mods. We give it speed (normally we use our normalspeed=0.95), Centimeters to drive
//    (we convert the centimeters to encoder ticks with a calculate , lines 40-41) and timeout (this is the maximum time that
//    this block will run. after this time we will move to the next block).
    public static void moveForward(double speed,
                                   double cmToDrive,
                                   double timeoutS, LinearVisionOpMode opMode, BlackTigersHardware robot, Telemetry telemetry) {
        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
//        We reset our encoders
        if (opMode.opModeIsActive()) {
            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // Determine new target position, and pass to motor controller
            int ticksToDrive = (int) (cmToDrive * COUNTS_PER_CM);
            newLeftTarget = robot.leftMotor.getCurrentPosition() + ticksToDrive;
            newRightTarget = robot.rightMotor.getCurrentPosition() + ticksToDrive;
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));
            telemetry.addData("power", speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {
                double percentage = (double) (ticksToDrive - (newLeftTarget - robot.leftMotor.getCurrentPosition())) / (double) ticksToDrive;
                if (percentage > 2) {
                    break;
                }
                double power = Math.abs(speed);
                if (power == 0) {
                    break;
                }
                robot.leftMotor.setPower(power);
                robot.rightMotor.setPower(power);
                // Display it for the driver.
//                write a telemetry
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.addData("gyro", robot.gyro.getHeading());
                telemetry.addData("Power", "left: %f, right: %f", robot.leftMotor.getPower(), robot.rightMotor.getPower());
                telemetry.update();
            }
// After the robot move to his new position we stop our motors
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);


            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }


    //We use a gyro sensor to make our robot rotates.
    public static void gyroRotate(int degrees, BlackTigersHardware robot, Telemetry telemetry, LinearVisionOpMode opMode) {
        int startPosition = robot.gyro.getHeading();
//        We find our target position
        int targetPosition = startPosition + degrees;
//      Here we find if our target position is more then 360 and if it more than 360 we substract 360.
        if (targetPosition >= 360) {
            targetPosition -= 360;
        } else if (targetPosition <= 0) {
            targetPosition += 360;
        }
//        We make our rotate until we are 2 degrees away from aur final position because we are terning
//        in high speed and in that way we prevent deviation of our robot.
//        Also we decelerate to be more precise with the gyro
        while (opMode.opModeIsActive() && Math.abs(targetPosition - robot.gyro.getHeading()) >= 2) {
            if (degrees > 0) {
                if (Math.abs((double) (targetPosition - robot.gyro.getHeading()) / degrees) < 0.25) {
                    robot.leftMotor.setPower(ROTATE_SPEED / 10);
                    robot.rightMotor.setPower(-ROTATE_SPEED / 10);
                } else {
                    robot.leftMotor.setPower(ROTATE_SPEED / 4);
                    robot.rightMotor.setPower(-ROTATE_SPEED / 4);
                }
            } else {
                if (Math.abs((double) (targetPosition - robot.gyro.getHeading()) / degrees) < 0.25) {
                    robot.leftMotor.setPower(-ROTATE_SPEED / 10);
                    robot.rightMotor.setPower(ROTATE_SPEED / 10);
                } else {
                    robot.leftMotor.setPower(-ROTATE_SPEED / 4);
                    robot.rightMotor.setPower(ROTATE_SPEED / 4);
                }
            }
            telemetry.addData("rotate0", "Starting Turn at %d", startPosition);
            telemetry.addData("rotate1", "Finishing Turn at %d", targetPosition);
            telemetry.addData("rotate2", "Robot gyro currently at %d", robot.gyro.getHeading());
            telemetry.update();
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    // At the start of our every Autonomous program we calebrate our gyro
    static public void calibrategyro(Telemetry telemetry, BlackTigersHardware robot, LinearVisionOpMode opMode) {
        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        robot.gyro.calibrate();

        robot.gyro.resetZAxisIntegrator();
        telemetry.addData(">", "Finished");
        telemetry.update();
    }

    //when we use our phones camera we need to config or camera settings
    public static void cameraSetup(LinearVisionOpMode opMode) {
        opMode.setCamera(Cameras.SECONDARY);
        opMode.setFrameSize(new Size(1080, 720));
        opMode.beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        opMode.beacon.setColorToleranceRed(0);
        opMode.beacon.setColorToleranceBlue(0);
        opMode.rotation.setIsUsingSecondaryCamera(true);
        opMode.rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);
        opMode.cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        opMode.cameraControl.setAutoExposureCompensation();
    }

}