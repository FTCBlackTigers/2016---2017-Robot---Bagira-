package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;


/**
 * Created by user on 29/12/2016.
 */
@Disabled
@Autonomous(name = "Black Tigers Blue Shoot&Beacon Auto", group = "BlackTigers Auto")
public class BlackTigersAutonomousBlueShootingBeacon extends LinearVisionOpMode {

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
        telemetry.addData(">", "Finished");

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForVisionStart();
        RobotUtilities.cameraSetup(this);
        enableExtension(VisionOpMode.Extensions.BEACON);
        enableExtension(VisionOpMode.Extensions.ROTATION);
        enableExtension(VisionOpMode.Extensions.CAMERA_CONTROL);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.addData("gyro", robot.gyro.getHeading());
        telemetry.update();


        waitForStart();
        robot.beaconsServo.setPosition(0.3);
        RobotUtilities.moveForward(1,10,1.5,this,robot,telemetry);

       if (runtime.seconds()<1.3){
        robot.reloadingMotor.setPower(1);
        robot.collectionMotor.setPower(1);
        robot.shootingMotor.setPower(1);}
        if (runtime.seconds()<2.8){
        robot.reloadingMotor.setPower(-1);
        robot.collectionMotor.setPower(1);
        robot.shootingMotor.setPower(1);}
        sleep(750);
        robot.reloadingMotor.setPower(0);
        robot.collectionMotor.setPower(1);
        robot.shootingMotor.setPower(1);
        sleep(1000);
        if (runtime.seconds()<4.46){
        robot.reloadingMotor.setPower(-1);
        robot.collectionMotor.setPower(1);
        robot.shootingMotor.setPower(1);}
        sleep(1000);
        robot.reloadingMotor.setPower(0);
        robot.collectionMotor.setPower(0);
        robot.shootingMotor.setPower(0);

        RobotUtilities.moveForward(0.95, 45, 3,this,robot,telemetry);
        RobotUtilities.gyroRotate(200,robot,telemetry, this);
        RobotUtilities.moveForward(0.95, -134.5, 6,this,robot,telemetry);
        RobotUtilities.gyroRotate (-25,robot,telemetry, this);
        RobotUtilities.moveForward(0.95,-90,3,this,robot,telemetry);
        RobotUtilities.gyroRotate(87,robot,telemetry, this);
        RobotUtilities.moveForward(0.95, -22, 2,this,robot,telemetry);

        RobotUtilities.pressBeacon(RobotUtilities.Color.RED, robot, beacon);
        RobotUtilities.moveForward(0.95, -55, 3,this,robot,telemetry); // Power:1 Distance:20 CM Time:2
      RobotUtilities.resetBeaconArm(robot);
        RobotUtilities.moveForward(1,40,1,this,robot,telemetry);
        RobotUtilities.gyroRotate(-45,robot,telemetry, this);
        RobotUtilities.moveForward(1,115,3,this,robot,telemetry);
      //  RobotUtilities.gyroRotate(25);
        //RobotUtilities.moveForward(1,25,25,2);





        while (opModeIsActive()) {

            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.update();
            waitOneFullHardwareCycle();
    }

}

}