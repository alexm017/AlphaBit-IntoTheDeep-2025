package org.firstinspires.ftc.teamcode.drive.Testers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeTesting {
    Servo left_torque;
    Servo right_torque;
    Servo y_rotation;
    Servo x_rotation;
    CRServo left_continous;
    CRServo right_continous;
    Servo intakeArm;
    Servo intakeArm2;
    Gamepad gmpd;
    Gamepad gmpd11;
    public IntakeTesting(HardwareMap hwdmap, Gamepad gmpd1){
        gmpd11 = gmpd1;
        intakeArm = hwdmap.get(Servo.class, "intakeArm");
        intakeArm2 = hwdmap.get(Servo.class, "intakeArm2");
        x_rotation = hwdmap.get(Servo.class, "XServo");
        y_rotation = hwdmap.get(Servo.class, "YServo"); // 3 EX
    }

    public double max_position = 1;
    public double min_position = 0;

    public double xpos;
    public double ypos;
    boolean togglebutton = false;
    boolean togglebuttondoi = false;

    public void Run(){
        if(gmpd11.left_bumper){
            if(!togglebutton) {
                max_position = max_position - 0.01;
                togglebutton = true;
            }
        }else if(gmpd11.right_bumper){
            if(!togglebuttondoi) {
                if (max_position < 1) {
                    max_position = max_position + 0.01;
                }
                togglebuttondoi = true;
            }
        }else if (gmpd11.left_trigger > 0.25){
            if(!togglebutton) {
                if (min_position > 0) {
                    min_position = min_position - 0.01;
                }
                togglebutton = true;
            }
        }else if(gmpd11.right_trigger > 0.25){
            if(!togglebuttondoi) {
                if (min_position < 1) {
                    min_position = min_position + 0.01;
                }
                togglebuttondoi = true;
            }
        }else {//0.9 ,     0.24
            togglebuttondoi = false;
            togglebutton = false;
        }

        if(gmpd11.y){
            y_rotation.setPosition(max_position);
        }else if(gmpd11.x){
            intakeArm.setPosition(max_position);
            intakeArm2.setPosition(max_position);
        }
    }
}
