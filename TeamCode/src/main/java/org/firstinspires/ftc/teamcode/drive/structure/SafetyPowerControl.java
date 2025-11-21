package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SafetyPowerControl {
    public DcMotorEx this_motor;
    boolean motor_has_encoders = true;
    boolean faulty_motor = false;
    boolean motor_power_given =false;

    double startTime = 0.00;
    double endTime = 0.00;
    ElapsedTime time = new ElapsedTime();

    public SafetyPowerControl(DcMotorEx motor){
        this_motor = motor;
    }

    public void init_motor(HardwareMap hwdmap, String motor_name, boolean is_reversed, boolean run_with_encoders, double multiplier){
        this_motor = hwdmap.get(DcMotorEx.class, motor_name);

    }

    public void isReversed(boolean is_rev){
        if(is_rev){
            this_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }else{
            this_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void runWithEncoders(boolean motor_has_enc){
        if(motor_has_enc){
            this_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }else{
            motor_has_encoders = false;
        }
    }
    public void SafetyPowerCheck(double power){
        if(motor_has_encoders) {
            if (!faulty_motor) {
                this_motor.setPower(power);
            } else {
                this_motor.setPower(0);
            }
            if (Math.abs(power) > 0.5 && !faulty_motor) {
                if (!motor_power_given) {
                    time.reset();
                    startTime = time.milliseconds();
                    motor_power_given = true;
                }
                if ((time.milliseconds() - startTime) > 2000) {
                    if (Math.abs(this_motor.getVelocity()) < 250) {
                        faulty_motor = true;
                    }
                }
            } else {
                motor_power_given = false;
            }
        }else{
            this_motor.setPower(power);
        }
    }

    public void SetMotorPower(double motor_power){
        SafetyPowerCheck(motor_power);
    }

    public boolean isMotorFaulty(){
        return faulty_motor;
    }

}
