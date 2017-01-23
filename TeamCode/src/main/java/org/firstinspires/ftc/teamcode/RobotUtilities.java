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
    static final double ROTATE_SPEED = 0.25;
    static final double WHEEL_DIAMETER_CM = 10.16;
    static final double CORRECTION_FACOTR = 0.15;
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    private static ElapsedTime runtime = new ElapsedTime();

    public static double normalizePower(double power) {
        return Range.clip(Math.pow(power,7), -0.7,0.7);
    }



    public static void moveForward(double speed,
                                   double cmToDrive,
                                   double timeoutS, LinearVisionOpMode opMode, BlackTigersHardware robot, Telemetry telemetry) {

        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
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

            int startDirection = robot.gyro.getHeading();

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())){
                int direction = robot.gyro.getHeading() - startDirection;
                double percentage = (double) (ticksToDrive - (newLeftTarget - robot.leftMotor.getCurrentPosition())) / (double) ticksToDrive;
                if(percentage > 1) {
                    break;
                }
                if(direction > 0) {
                    if(direction <= 180) {
                        robot.leftMotor.setPower(Range.clip(getPowerToDrive(Math.abs(speed), percentage) - getErrorFraction(direction), 0,1));
                    } else {
                        direction = Math.abs(direction -360);
                        robot.rightMotor.setPower(Range.clip(getPowerToDrive(Math.abs(speed), percentage) - getErrorFraction(direction), 0, 1));
                    }
                } else {
                    double power = getPowerToDrive(Math.abs(speed), percentage);
                    if(power == 0) {
                        break;
                    }
                    robot.leftMotor.setPower(power);
                    robot.rightMotor.setPower(power);
                }
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.addData("Power", "left: %f, right: %f", robot.leftMotor.getPower(), robot.rightMotor.getPower());
                telemetry.update();
            }

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    public static double getPowerToDrive(double maxSpeed, double percentageOfDistance) {
        if(percentageOfDistance < 0.75) {
            return maxSpeed;
        } else {
            double speed = (1-Math.sqrt(percentageOfDistance)) * maxSpeed;
            if(speed < 0.03) {
                return 0.1;
            }
            return speed;
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
        while(opMode.opModeIsActive() && Math.abs(targetPosition-robot.gyro.getHeading()) > 1) {
//            while(opMode.opModeIsActive() && Math.abs(targetPosition-robot.gyro.getHeading()) > 3) {
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
