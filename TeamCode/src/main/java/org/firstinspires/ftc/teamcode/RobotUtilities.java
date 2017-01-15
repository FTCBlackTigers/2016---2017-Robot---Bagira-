package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by user on 05/11/2016.
 */

public class RobotUtilities {

    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 3.8;
    static final double ROTATE_SPEED = 0.5;
    static final double WHEEL_DIAMETER_CM = 10.16;
    static final double CORRECTION_FACOTR = 0.15;
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    private static ElapsedTime runtime = new ElapsedTime();

    public static double normalizePower(double power) {
        return Math.pow(power,3);
    }

    public static void encoderDrive(double speed,
                             double leftCM, double rightCM,
                             double timeoutS, LinearVisionOpMode opMode, BlackTigersHardware robot, Telemetry telemetry) {

        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
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
            while (opMode.opModeIsActive() &&
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
            while(opMode.opModeIsActive() && i < 4 && runtime.milliseconds() % 20 == 0) {
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

    public static double getErrorFraction(int direction) {
        return Range.clip(direction * CORRECTION_FACOTR, 0, 1);
    }

    public static void gyroRotate(int degrees, BlackTigersHardware robot, Telemetry telemetry, LinearVisionOpMode opMode) {
        //degrees=-degrees; // gyro is backwards
        int startPosition = robot.gyro.getHeading();
        int targetPosition = startPosition+degrees;
        if(targetPosition > 360) {
            targetPosition -= 360;
        } else if (targetPosition < 0) {
            targetPosition += 360;
        }
        while(opMode.opModeIsActive() && Math.abs(targetPosition-robot.gyro.getHeading()) > 3) {
            if(degrees > 0) {
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
