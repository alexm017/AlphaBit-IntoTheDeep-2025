package org.firstinspires.ftc.teamcode.drive.Testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlidersPositionTesting {
    DcMotorEx leftSlider;
    DcMotorEx rightSlider;

    Gamepad gmpd;

    public SlidersPositionTesting(HardwareMap hwdmap, Gamepad gmpdd){
        gmpd = gmpdd;
        leftSlider = hwdmap.get(DcMotorEx.class, "LeftSlider");
        rightSlider = hwdmap.get(DcMotorEx.class, "RightSlider");

        leftSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlider.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double leftSliderPos;
    public double rightSliderPos;

    public void Run(){
        leftSliderPos = leftSlider.getCurrentPosition();
        rightSliderPos = rightSlider.getCurrentPosition();

        if(gmpd.right_bumper){
            leftSlider.setPower(0.4);
            rightSlider.setPower(0.4);
        }else if(gmpd.left_bumper){
            leftSlider.setPower(-0.4);
            rightSlider.setPower(-0.4);
        }else if(gmpd.x){
            leftSlider.setPower(0.4);
        }else if(gmpd.b){
            rightSlider.setPower(0.4);
        }else{
            leftSlider.setPower(0);
            rightSlider.setPower(0);
        }
    }
}
