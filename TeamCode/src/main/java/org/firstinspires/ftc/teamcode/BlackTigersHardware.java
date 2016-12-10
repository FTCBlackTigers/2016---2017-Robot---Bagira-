package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BlackTigersHardware
{
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;



    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public BlackTigersHardware(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");

        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors


        leftMotor.setPower(0);
        rightMotor.setPower(0);



        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}

