package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class BlackTigersHardwareYoav
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  collectionMotor = null;
    public DcMotor  reloadingMotor = null;
    public DcMotor  shootingMotor = null;




    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    public BlackTigersHardwareYoav(){

    }


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;


        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        collectionMotor = hwMap.dcMotor.get("collection_motor");
        reloadingMotor = hwMap.dcMotor.get("reloading_motor");
        shootingMotor = hwMap.dcMotor.get("shooting_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        collectionMotor.setDirection(DcMotor.Direction.REVERSE);
        reloadingMotor.setDirection(DcMotor.Direction.REVERSE);
        shootingMotor.setDirection(DcMotor.Direction.REVERSE);


        leftMotor.setPower(0);
        rightMotor.setPower(0);
        collectionMotor.setPower(0);
        reloadingMotor.setPower(0);
        shootingMotor.setPower(0);




        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        reloadingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}

