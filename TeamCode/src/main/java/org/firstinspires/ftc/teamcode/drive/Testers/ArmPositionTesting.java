package org.firstinspires.ftc.teamcode.drive.Testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmPositionTesting {
    DcMotorEx armPos;

    Gamepad gmpd;

    public ArmPositionTesting(HardwareMap hwdmap, Gamepad gmpdd){
        gmpd = gmpdd;
        armPos = hwdmap.get(DcMotorEx.class, "UpArm");
        //DownArm IS REVERSED MY NIGU
        //UpArm is FORWARD MY NIGU

        armPos.setDirection(DcMotorSimple.Direction.FORWARD);
        armPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPos.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double armpos;
    public void Run(){
        armpos = armPos.getCurrentPosition();
        if(gmpd.dpad_up){
            armPos.setPower(1);
        }else if(gmpd.dpad_down){
            armPos.setPower(-1);
        }else{
            armPos.setPower(0);
        }
    }
}
