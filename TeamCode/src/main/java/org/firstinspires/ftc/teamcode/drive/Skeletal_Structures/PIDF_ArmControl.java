package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF_ArmControl {

    DcMotorEx this_motor;

    ElapsedTime timer = new ElapsedTime();

    public double Kp_;
    public double Ki_;
    public double Kd_;
    public double Kf_;

    double IntegralSum = 0;
    double lastError = 0;
    public double primaryOutput;
    public double primaryDerivative;

    public PIDF_ArmControl(DcMotorEx x1){
        this_motor = x1;
    }

    public void SetPIDFCoefficients(double Kp, double Kd, double Ki, double Kf){
        Kp_ = Kp;
        Kd_ = Kd;
        Ki_ = Ki;
        Kf_ = Kf;
    }

    public void motor_init(HardwareMap hwdmap, String motor_name, boolean is_reversed, boolean run_with_encoders, double multiplier){
        this_motor = hwdmap.get(DcMotorEx.class, motor_name);

        IsReversed(is_reversed);

        RunWithEncoders(run_with_encoders);

        this_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void IsReversed(boolean x1){
        if(x1){
            this_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }else{
            this_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void RunWithEncoders(boolean x1){
        if(x1){
            this_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            this_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public double GetPIDFPower(double reference_){
        double current_position = this_motor.getCurrentPosition();
        double error = reference_ - current_position;
        IntegralSum += error * timer.seconds();
        primaryDerivative = ((error - lastError) / timer.seconds()) * Kd_;
        lastError = error;

        timer.reset();

        primaryOutput = ((error * Kp_) + (primaryDerivative));
        return Math.max(-1, Math.min(1, primaryOutput));
    }

    public void SetPIDPower(double reference_point){
        double power = GetPIDFPower(reference_point);
        this_motor.setPower(power);
    }

    public double MotorPosition(){
        return this_motor.getCurrentPosition();
    }

    public double MotorVelocity() {
        return this_motor.getVelocity();
    }
    public void SetMotorPower(double x1){
        this_motor.setPower(x1);
    }
}
