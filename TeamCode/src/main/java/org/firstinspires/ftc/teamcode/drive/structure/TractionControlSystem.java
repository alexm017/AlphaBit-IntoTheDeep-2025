package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TractionControlSystem {
    //SampleMecanumDrive drive;
    Telemetry telemetry;
    //TwoWheelTrackingLocalizer robotPositionControl;
    ChasisInit chasisControl;
    Gamepad traction_gamepad;

    public TractionControlSystem(HardwareMap hwdmap, Telemetry telemetrys, Gamepad gmpd){
        telemetry = telemetrys;
        traction_gamepad = gmpd;
        //drive = new SampleMecanumDrive(hwdmap);
        //robotPositionControl = new TwoWheelTrackingLocalizer(hwdmap, drive);
        chasisControl = new ChasisInit(hwdmap);
    }

    boolean TCS_Toggle = false;
    public double x_pos;
    public double y_pos;
    public double x_velocity;
    public double y_velocity;
    public double BackLeft_Velocity;
    public double FrontLeft_Velocity;

    double slipThreshold = 5;
    boolean slipDetected = false;
    boolean TCS_Status = false;
    public double FLSlipRatio;

    public void Run(){
        /*x_pos = robotPositionControl.encoderTicksToInches(robotPositionControl.parallelEncoder.getCurrentPosition());
        y_pos = robotPositionControl.encoderTicksToInches(robotPositionControl.perpendicularEncoder.getCurrentPosition());
        x_velocity = robotPositionControl.parallelEncoder.getCorrectedVelocity();
        y_velocity = robotPositionControl.perpendicularEncoder.getCorrectedVelocity();*/
        BackLeft_Velocity = chasisControl.FrontRightVelocity();
        FrontLeft_Velocity = chasisControl.BackRightVelocity();

        if(traction_gamepad.y){
            if(!TCS_Toggle){
                if(!TCS_Status){
                    TCS_Status = true;
                }else{
                    TCS_Status = false;
                }
                TCS_Toggle = true;
            }
        }else{
            TCS_Toggle = false;
        }

        if(TCS_Status){
            TCS_SlipDetection();
        }
    }

    public void TCS_SlipDetection(){
        FLSlipRatio = Math.abs(x_velocity/FrontLeft_Velocity);
         
        if(FLSlipRatio < slipThreshold){
            slipDetected = true;
            telemetry.addData("[!!!] FrontLeft SLIP DETECTED[!!!]", slipDetected);
            traction_gamepad.rumble(100);
        }else{
            slipDetected = false;
        }
    }

    public String TCS_Status(){
        if(TCS_Status){
            return "On";
        }else{
            return "Off";
        }
    }
}
