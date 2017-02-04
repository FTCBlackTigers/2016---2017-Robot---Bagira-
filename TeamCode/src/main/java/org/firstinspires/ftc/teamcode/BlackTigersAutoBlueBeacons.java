package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
/*
 * Created by user on 01/02/2017.
 */


@Autonomous(name = "BlackTigersAutoBlueBeacons", group = "BlackTigers Auto")
    public class BlackTigersAutoBlueBeacons extends LinearVisionOpMode {

    BlackTigersHardware robot = new BlackTigersHardware();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Calibrating Gyro");

        robot.gyro.calibrate();

        robot.gyro.resetZAxisIntegrator();
        telemetry.addData(">", "Finished");


        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForVisionStart();
       /* RobotUtilities.cameraSetup(this);
        enableExtension(VisionOpMode.Extensions.BEACON);
        enableExtension(VisionOpMode.Extensions.ROTATION);
        enableExtension(VisionOpMode.Extensions.CAMERA_CONTROL);
*/


        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();


        waitForStart();

        RobotUtilities.moveForward(RobotUtilities.normalSpeed, 200,10, this, robot, telemetry);













        while (opModeIsActive()) {

            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.update();
//

            waitOneFullHardwareCycle();
        }

    }
}