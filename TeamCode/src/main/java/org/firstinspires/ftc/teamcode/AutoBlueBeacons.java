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
        robot.init(hardwareMap);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RobotUtilities.calibrategyro(telemetry , robot ,this);
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

        RobotUtilities.moveForward(RobotUtilities.normalSpeed, -38,10, this, robot, telemetry);
        RobotUtilities.gyroRotate(37, robot, telemetry, this);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed, -90,10, this, robot, telemetry);
        sleep(200);
        RobotUtilities.gyroRotate(-35, robot, telemetry, this);
        sleep(100);
        while(!beacon.getAnalysis().isRightKnown() || !beacon.getAnalysis().isLeftKnown()) {

        }
        boolean isBlueRight = beacon.getAnalysis().isRightBlue();
        telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
        telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
        telemetry.update();
        RobotUtilities.moveForward(RobotUtilities.normalSpeed/3, -4,10, this, robot, telemetry);
        sleep(200);
        RobotUtilities.gyroRotate(90, robot, telemetry, this);
        robot.shootingMotor.setPower(0.75);
        sleep(200);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed/2, -63,10, this, robot, telemetry);
        RobotUtilities.moveForward(0.35, 40,10, this, robot, telemetry);
        sleep(1500);
        robot.reloadingMotor.setPower(-0.75);
        sleep(2500);
        robot.shootingMotor.setPower(0);
        robot.reloadingMotor.setPower(0);
        if(!isBlueRight) {
            RobotUtilities.moveForward(RobotUtilities.normalSpeed/2, -58 ,10, this, robot, telemetry);
            sleep(700);
            RobotUtilities.moveForward(RobotUtilities.normalSpeed, 35 ,10, this, robot, telemetry);
        }
        sleep(500);
        RobotUtilities.gyroRotate(-85, robot, telemetry, this);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed, -103,10, this, robot, telemetry);
        sleep(500);
        isBlueRight = beacon.getAnalysis().isRightBlue();
        telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
        telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
        if(!isBlueRight) {
            RobotUtilities.moveForward(RobotUtilities.normalSpeed,-15,5,this,robot,telemetry);
        }
       RobotUtilities.gyroRotate(85, robot, telemetry, this);
        sleep(200);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed/2, -66 ,10, this, robot, telemetry);
        RobotUtilities.moveForward(RobotUtilities.normalSpeed/3, 5 ,10, this, robot, telemetry);


        while (opModeIsActive()) {
            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            waitOneFullHardwareCycle();
        }
    }
}