package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF_SlidersControl {

    public double Kp_Slider;
    public double Kd_Slider;
    public double Ki_Slider;
    public double Kf_Slider;

    double IntegralSum;
    double lastError;
    double leftMultiplier = 1;
    double rightMultiplier = 1;
    public double slidersPower;

    static double sliders_limit = 2750;
    public double error = 0.00;

    ElapsedTime timer = new ElapsedTime();

    DcMotorEx First_Motor;
    DcMotorEx Second_Motor;
    DcMotorEx Third_Motor;

    public PIDF_SlidersControl(DcMotorEx FirstMotor, DcMotorEx SecondMotor, DcMotorEx ThirdMotor){
        First_Motor = FirstMotor;
        Second_Motor = SecondMotor;
        Third_Motor = ThirdMotor;
    }

    public void motors_init(HardwareMap hwdmap, String motor_name1, boolean reversed1, boolean using_encoders1, double multiply1, String motor_name2, boolean reversed2, boolean using_encoders2, double multiply2, String motor_name3, boolean reversed3, boolean using_encoders3, double multiply3){

        First_Motor = hwdmap.get(DcMotorEx.class, motor_name1);
        Second_Motor = hwdmap.get(DcMotorEx.class, motor_name2);
        Third_Motor = hwdmap.get(DcMotorEx.class, motor_name3);

        leftMultiplier = multiply1;
        rightMultiplier = multiply2;

        First_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Second_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Third_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ReversedMotors_Check(reversed1, reversed2, reversed3);

        UsingEncoders_Check(using_encoders1, using_encoders2, using_encoders3);
    }

    public void ReversedMotors_Check(boolean rev1, boolean rev2, boolean rev3){
        if(rev1) {
            First_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }else {
            First_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        if(rev2){
            Second_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }else {
            Second_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        if(rev3){
            Third_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }else {
            Third_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void UsingEncoders_Check(boolean us1, boolean us2, boolean us3){
        if(us1){
            First_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            First_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(us2){
            Second_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            Second_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(us3){
            Third_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            Third_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void SetPidfCoefficients(double kp_, double kd_, double ki_, double kf_){
        Kp_Slider = kp_;
        Kd_Slider = kd_;
        Ki_Slider = ki_;
        Kf_Slider = kf_;
    }

    public void PIDF_Raw_Power_Output(double reference){
        double current_position = Second_Motor.getCurrentPosition();
        error = reference - current_position;
        IntegralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        if(lastError < 10){
            IntegralSum = 0;
        }

        double ff_power = Kf_Slider;

        timer.reset();

        slidersPower = (error * Kp_Slider) + (derivative * Kd_Slider) + (IntegralSum * Ki_Slider) + ff_power;

        slidersPower = Math.max(-1, Math.min(1, slidersPower));
    }

    public void SetPIDFPower(double ref){
        if(First_Motor.getCurrentPosition() < sliders_limit && Second_Motor.getCurrentPosition() < sliders_limit) {
            PIDF_Raw_Power_Output(ref);
            First_Motor.setPower(slidersPower * 1);
            Second_Motor.setPower(slidersPower * 1);
            Third_Motor.setPower(slidersPower * 1);
        }else{
            First_Motor.setPower(0);
            Second_Motor.setPower(0);
            Third_Motor.setPower(0);
        }
    }

    public void SetControledPower(double motor_power){
        if(Math.abs(First_Motor.getCurrentPosition()) < sliders_limit && Math.abs(Second_Motor.getCurrentPosition()) < sliders_limit){
            First_Motor.setPower(motor_power);
            Second_Motor.setPower(motor_power);
            Third_Motor.setPower(motor_power);
        }else{
            First_Motor.setPower(0);
            Second_Motor.setPower(0);
            Third_Motor.setPower(0);
        }
    }

    public double GetSliderReference_Manual(){
        double slider_reference = 0;
        if(First_Motor.getCurrentPosition() > Second_Motor.getCurrentPosition()){
            slider_reference = First_Motor.getCurrentPosition();
        }else{
            slider_reference = Second_Motor.getCurrentPosition();
        }
        return slider_reference;
    }

    public boolean MotorsStopped(){
        boolean motors_stopped = false;
        if(abs(MotorsVelocity()) < 300){
            motors_stopped = true;
        }
        return motors_stopped;
    }

    public double LeftMotorVelocity(){
        return First_Motor.getVelocity();
    }

    public double RightMotorVelocity(){
        return Second_Motor.getVelocity();
    }

    public double MotorsVelocity(){
        return (First_Motor.getVelocity() + Second_Motor.getVelocity())/2;
    }

    public double LeftMotorCurrentPosition(){
        return First_Motor.getCurrentPosition();
    }

    public double RightMotorCurrentPosition(){
        return Second_Motor.getCurrentPosition();
    }
    public double ThirdMotorCurrentPosition(){ return Third_Motor.getCurrentPosition();}
}
