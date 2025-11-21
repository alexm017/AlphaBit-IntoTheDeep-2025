package org.firstinspires.ftc.teamcode.drive.structure;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.network.PreferenceRemoter;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.PIDF_ArmControl;

@Config
public class ArmControl {
    DcMotorEx ArmMotor;
    PIDF_ArmControl armControl = new PIDF_ArmControl(ArmMotor);
    Gamepad gamepad_arm;
    SlidersControl slidersControl;
    IntakeControl intakeControl;

    public static double Kp = 0.00325;
    public static double Ki = 0.00;
    public static double Kd = 0.000125;
    public static double Kf = 0.00;

    public static double sKp = 0.001;
    public static double sKi = 0.00;
    public static double sKd = 0.00000009;
    public static double sKf = 0.00;

    public double reference = 0.00;
    public double auto_reference = 0.00;
    Gamepad temp_gmpd;

    public ArmControl(HardwareMap hwdmap, Gamepad gmpd1, Gamepad gmpd2){
        armControl.motor_init(hwdmap, "Arm", false, true, 1);
        armControl.SetPIDFCoefficients(Kp, Kd, Ki, Kf);

        gamepad_arm = gmpd1;
        temp_gmpd = gmpd2;
    }

    public void slidersInit(SlidersControl slidersControlf){
        slidersControl = slidersControlf;
    }

    public void intakeInit(IntakeControl intakeControll){
        intakeControl = intakeControll;
    }
    public double arm_position;
    public double arm_velocity;

    public double defaultThreshold = 25;
    boolean button_pressed = false;
    boolean arm_is_up = false;
    boolean pickup_specimen = false;
    boolean place_specimen = false;

    boolean waiting_to_pick_specimen = false;
    boolean lock_intake_controls = false;
    boolean waiting_to_place_specimen = false;
    boolean specimen_has_been_placed = false;
    boolean checkMode = false;
    boolean hasFinished = true;
    public double getReferencePosition;
    boolean buttonBeenPressed = false;
    public double getPrimaryOutput;
    public double getDerivativeOutput;
    boolean wantsToHang = false;
    public boolean hasReachedReference = false;
    boolean hasReachedAutoReference = false;
    boolean isArmCalibrated = false;
    double maxArmPos = 0;

    public void Run(){
        getDerivativeOutput = armControl.primaryDerivative;
        getPrimaryOutput = armControl.primaryOutput;
        getReferencePosition = reference;
        arm_position = armControl.MotorPosition();
        arm_velocity = armControl.MotorVelocity();

        if(gamepad_arm.left_stick_button){
            if(!buttonBeenPressed) {
                if (!wantsToHang) {
                    wantsToHang = true;
                    reference = 550;
                    intakeControl.getSpecimen(true);
                }else{
                    wantsToHang = false;
                    reference = 0;
                    intakeControl.init();
                }
                buttonBeenPressed = true;
            }
        }else{
            buttonBeenPressed = false;
        }

        if(gamepad_arm.dpad_up && temp_gmpd.left_trigger < 0.5){
            if(!button_pressed){
                button_pressed = true;
                if(!place_specimen){
                    place_specimen = true;
                }else{
                    place_specimen = false;
                }
                arm_is_up = false;
                pickup_specimen = false;
            }
        }else if(gamepad_arm.dpad_down && temp_gmpd.left_trigger < 0.5){
            if(!button_pressed) {
                button_pressed = true;
                if(!pickup_specimen){
                    pickup_specimen = true;
                }else{
                    pickup_specimen = false;
                }
                arm_is_up = false;
                place_specimen = false;
            }
        }else if(gamepad_arm.dpad_left && temp_gmpd.left_trigger < 0.5){
            if(!button_pressed) {
                if (!arm_is_up) {
                    arm_is_up = true;
                } else {
                    arm_is_up = false;
                }
                button_pressed = true;
                pickup_specimen = false;
                place_specimen = false;
            }
        }else{
            button_pressed = false;
        }
        if(!place_specimen && !arm_is_up && !pickup_specimen){
            if(hasFinished) {
                checkMode = false;
            }
        }else{
            checkMode = true;
            hasFinished = false;
        }

        if(checkMode && !wantsToHang) {
            if (armControl.MotorPosition() < 800 && place_specimen && slidersControl.slidersDown() && !pickup_specimen && !arm_is_up) {
                reference = 870;
                intakeControl.placeSpecimen(true);
                lock_intake_controls = true;
                waiting_to_place_specimen = false;
            } else if (armControl.MotorPosition() < 800 && place_specimen && !slidersControl.slidersDown() && !arm_is_up && !pickup_specimen) {
                slidersControl.SetToReference(0);
                intakeControl.placeSpecimen(true);
                lock_intake_controls = true;
                waiting_to_place_specimen = false;
            } else if (armControl.MotorPosition() > 800 && place_specimen && !waiting_to_place_specimen && slidersControl.slidersDown() && !arm_is_up && !pickup_specimen) {
                slidersControl.SetToReference(200);
                lock_intake_controls = true;
                waiting_to_place_specimen = true;
            } else if (armControl.MotorPosition() > 800 && !place_specimen && !slidersControl.slidersDown() && !arm_is_up && !pickup_specimen) {
                slidersControl.SetToReference(0);
                lock_intake_controls = true;
                waiting_to_place_specimen = false;
                specimen_has_been_placed = false;
            } else if (specimen_has_been_placed && waiting_to_place_specimen && place_specimen && !arm_is_up && !pickup_specimen){
                if(slidersControl.SlidersPosition > 400){
                    specimen_has_been_placed = false;
                    intakeControl.ClawControl(true);
                }
            }else if (gamepad_arm.y && waiting_to_place_specimen && place_specimen && !arm_is_up && !pickup_specimen) {
                slidersControl.SetToReference(650);
                specimen_has_been_placed = true;
                lock_intake_controls = true;
            }else if (armControl.MotorPosition() > 800 && !place_specimen && slidersControl.slidersDown() && !arm_is_up && !pickup_specimen) {
                intakeControl.placeSpecimen(false);
                reference = 0;
                lock_intake_controls = false;
                waiting_to_place_specimen = false;
                specimen_has_been_placed = false;
                hasFinished = true;
            }

            if (pickup_specimen && slidersControl.slidersDown() && !arm_is_up && !place_specimen) {
                intakeControl.getSpecimen(true);
                waiting_to_pick_specimen = true;
                lock_intake_controls = true;
            } else if (pickup_specimen && !slidersControl.slidersDown() && !arm_is_up && !place_specimen) {
                slidersControl.SetToReference(0);
                waiting_to_pick_specimen = false;
                lock_intake_controls = true;
            } else if (!pickup_specimen && waiting_to_pick_specimen && !arm_is_up && !place_specimen) {
                intakeControl.getSpecimen(false);
                waiting_to_pick_specimen = false;
                lock_intake_controls = false;
                hasFinished = true;
            }

            if (!isArmUp() && arm_is_up && slidersControl.slidersDown() && !pickup_specimen && !place_specimen) {
                reference = 870;
                intakeControl.straightBasket();
                lock_intake_controls = true;
            } else if (!isArmUp() && arm_is_up && !slidersControl.slidersDown() && !pickup_specimen && !place_specimen) {
                slidersControl.SetToReference(0);
                intakeControl.scoreBasketSample(false);
                lock_intake_controls = true;
            } else if (isArmUp() && slidersControl.slidersDown() && arm_is_up && !pickup_specimen && !place_specimen) {
                slidersControl.SetToReference(2450);
                lock_intake_controls = true;
            } else if (isArmUp() && slidersControl.hasReachedReference() && slidersControl.getReference() == 2450 && arm_is_up && !pickup_specimen && !place_specimen) {
                intakeControl.scoreBasketSample(true);
                lock_intake_controls = true;
            } else if (isArmUp() && !slidersControl.slidersDown() && !arm_is_up && !pickup_specimen && !place_specimen) {
                intakeControl.straightBasket();
                slidersControl.SetToReference(0);
                lock_intake_controls = true;
            } else if (isArmUp() && slidersControl.slidersDown() && !arm_is_up && !pickup_specimen && !place_specimen) {
                intakeControl.scoreBasketSample(false);
                reference = 0;
                lock_intake_controls = false;
                hasFinished = true;
            }
        }

        if (!slidersControl.hasReachedReference()) {
            slidersControl.AutonomousUpdate();
        }

        if(Math.abs(reference - arm_position) < 5 && !hasReachedReference){
            hasReachedReference = true;
            armControl.SetMotorPower(0);
        }else if(Math.abs(reference - arm_position) > 10){
            hasReachedReference = false;
        }

        if(!hasReachedReference) {
            armControl.SetPIDPower(reference);
        }
    }

    public boolean isArmUp(){
        if(armControl.MotorPosition() > 800){
            return true;
        }else{
            return false;
        }
    }

    public void SetToReference(double ref){
        auto_reference = ref;
        reference = ref;
    }

    public void AutonomousUpdate(){
        arm_position = armControl.MotorPosition();
        arm_velocity = armControl.MotorVelocity();

        if(Math.abs(auto_reference - arm_position) < 10 && !hasReachedAutoReference){
            hasReachedAutoReference = true;
            armControl.SetMotorPower(0);
        }else if(Math.abs(auto_reference - arm_position) > 15){
            hasReachedAutoReference = false;
        }

        if(!hasReachedAutoReference) {
            armControl.SetPIDPower(auto_reference);
        }
    }

    public void setCustomThreshold(double custom_threshold){
        defaultThreshold = custom_threshold;
    }

    public boolean isBusy(){
        if(Math.abs(auto_reference - armControl.MotorPosition()) > defaultThreshold){
            return true;
        }else{
            return false;
        }
    }

    public boolean isManualBusy(){
        if(Math.abs(reference - armControl.MotorPosition()) > defaultThreshold){
            return true;
        }else{
            return false;
        }
    }
}
