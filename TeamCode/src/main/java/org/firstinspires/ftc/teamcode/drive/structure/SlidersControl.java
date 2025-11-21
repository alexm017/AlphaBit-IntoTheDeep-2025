package org.firstinspires.ftc.teamcode.drive.structure;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.PIDF_SlidersControl;

@Config
public class SlidersControl {
    DcMotorEx leftSlider;
    DcMotorEx rightSlider;
    DcMotorEx thirdSlider;
    Gamepad gamepad_sliders;
    IntakeControl intakeControl;
    PIDF_SlidersControl slidersControl = new PIDF_SlidersControl(leftSlider, rightSlider, thirdSlider);
    ArmControl armControl;
    ElapsedTime timer = new ElapsedTime();
    Telemetry telemetry;

    public static double Kp = 0.003;
    public static double Ki = 0.00;
    public static double Kd = 0.000045;
    public static double Kf = 0.008;

    public SlidersControl(HardwareMap hwdmap, Gamepad gmpd, ArmControl armControls){
        armControl = armControls;
        gamepad_sliders = gmpd;
        slidersControl.motors_init(hwdmap, "LeftSlider", false, true, 0.5, "RightSlider",  false, true, 0.5, "ThirdSlider", true, true, 0.5);

        slidersControl.SetPidfCoefficients(Kp, Kd, Ki, Kf);
    }

    public void intakeInit(IntakeControl intakeControll){
        intakeControl = intakeControll;
    }

    public double SlidersPosition;
    public double SlidersVelocity;
    public double LeftSliderPos;
    public double RightSliderPos;

    public double defaultThreshold = 100;
    public double reference = 0.00;
    enum SlidersState{
        Stationary,
        Autonomous,
        ManualControl,
        Moving
    }

    double down_reference = 0;

    double max_horziontal_limit = 800;
    double horizontal_reference = 750;
    SlidersState slidersState = SlidersState.Stationary;

    public double leftVelocity;
    public double rightVelocity;

    boolean slidersUp = false;
    boolean button_already_pressed = false;
    boolean isArmup;

    double max_extension_limit = 2500;
    boolean slidersStartMoving = false;
    double slidersPositionCompensation = 0.00;
    public boolean isPositionCompensationActivated = false;
    boolean manual_control = false;
    public boolean slidersFaultDetected = false;
    public double thirdsliderpos;

    public boolean isNigger = false;
    boolean subCheck = false;
    boolean isCheckCalled = false;

    public void Run(){
        isArmup = armControl.isArmUp();
        leftVelocity = slidersControl.LeftMotorVelocity();
        rightVelocity = slidersControl.RightMotorVelocity();
        SlidersVelocity = slidersControl.MotorsVelocity();
        SlidersPosition = (slidersControl.LeftMotorCurrentPosition() + slidersControl.RightMotorCurrentPosition())/2;
        LeftSliderPos = slidersControl.LeftMotorCurrentPosition();
        RightSliderPos = slidersControl.RightMotorCurrentPosition();
        thirdsliderpos = slidersControl.ThirdMotorCurrentPosition();
        isNigger = hasReachedReference();

        if(gamepad_sliders.right_bumper && SlidersPosition < max_horziontal_limit && !isArmup && slidersState != SlidersState.Autonomous){
            slidersControl.SetControledPower(1);
            slidersState = SlidersState.ManualControl;
            manual_control = true;
        }else if(gamepad_sliders.right_bumper && SlidersPosition < max_extension_limit && isArmup && slidersState != SlidersState.Autonomous){
            slidersControl.SetControledPower(1);
            slidersState = SlidersState.ManualControl;
            manual_control = true;
        }else if(gamepad_sliders.left_bumper && SlidersPosition > 0 && slidersState != SlidersState.Autonomous){
            slidersControl.SetControledPower(-1);
            slidersState = SlidersState.ManualControl;
            manual_control = true;
        }else if(slidersState == SlidersState.ManualControl){
            slidersControl.SetControledPower(0);
            slidersState = SlidersState.Moving;
        }

        if(slidersState == SlidersState.Moving && SlidersVelocity < 5){
            reference = SlidersPosition;
            slidersState = SlidersState.Stationary;
        }

        if(gamepad_sliders.dpad_right && slidersState == SlidersState.Stationary){
            if(!button_already_pressed) {
                if (!slidersUp) {
                    reference = horizontal_reference;
                    manual_control = false;
                    slidersStartMoving = true;
                    slidersUp = true;
                    subCheck = false;
                } else {
                    reference = 0;
                    subCheck = true;
                    slidersStartMoving = true;
                    manual_control = false;
                    slidersUp = false;
                }
                slidersState = SlidersState.Autonomous;
                gamepad_sliders.rumble(500);
                button_already_pressed = true;
            }
        }else{
            button_already_pressed = false;
        }

        if(subCheck){
            if(!isCheckCalled) {
                intakeControl.safeInit();
                isCheckCalled = true;
            }
            if(reference == 0 && SlidersPosition < 50){
                subCheck = false;
                isCheckCalled = false;
                intakeControl.init();
            }
        }

        if(slidersState == SlidersState.Autonomous) {
            if (reference == horizontal_reference && ((horizontal_reference - SlidersPosition) < 250) || ((horizontal_reference - LeftSliderPos) < 100)) {
                slidersState = SlidersState.Stationary;
            } else if (reference == down_reference && ((SlidersPosition - down_reference) < 250) || ((LeftSliderPos - down_reference) < 100)) {
                slidersState = SlidersState.Stationary;
            }
        }

        if(slidersState == SlidersState.Stationary || slidersState == SlidersState.Autonomous){
            slidersControl.SetPIDFPower(reference);
        }
    }

    public boolean slidersDown(){
        if(SlidersPosition < 75){
            return true;
        }else{
            return false;
        }
    }

    public void getSampleCustomDistance(double distanceInCm, boolean inSubmersible){
        double distanceInTicks;
        if(!inSubmersible){
            distanceInTicks = 315.00/8.00;
        }else{
            distanceInTicks = 385.00/8.00;
        }
        double setToReferenceDistance = reference + (distanceInCm * distanceInTicks);
        SetToReference(setToReferenceDistance);
    }

    public boolean hasReachedReference(){
        if(slidersStartMoving){
            if(SlidersVelocity > 50){
                timer.reset();
            }
        }
        if((Math.abs(reference - SlidersPosition) < 35)){
            slidersStartMoving = false;
            return true;
        }else if(SlidersVelocity < 50 && (Math.abs(reference - SlidersPosition) > 30) && timer.milliseconds() > 550 && !manual_control){
            slidersPositionCompensation = Math.abs(SlidersPosition - reference);
            isPositionCompensationActivated = true;
            slidersStartMoving = false;
            return true;
        }else{
            return false;
        }
    }
    public double getReference(){
        return reference;
    }
    public void SetToReference(double reference_){
        reference = reference_;
        slidersStartMoving = true;
        manual_control = false;
        if(reference_ == 0){
            slidersUp = false;
        }
    }
    public void AutonomousUpdate(){
        SlidersVelocity = slidersControl.MotorsVelocity();
        SlidersPosition = (slidersControl.LeftMotorCurrentPosition() + slidersControl.RightMotorCurrentPosition())/2;
        if(slidersState == SlidersState.Stationary) {
            slidersControl.SetPIDFPower(reference);
        }
        manual_control = false;
    }

    public void setCustomThreshold(double custom_threshold){
        defaultThreshold = custom_threshold;
    }

    public boolean isBusy(){
        double avgPosition = (slidersControl.LeftMotorCurrentPosition() + slidersControl.RightMotorCurrentPosition())/2;
        if(Math.abs(reference - avgPosition) > defaultThreshold){
            return true;
        }else{
            return false;
        }
    }
}
