package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriverAttentionControl {
    ElapsedTime elapsedTime = new ElapsedTime();
    Gamepad gmpd;

    public DriverAttentionControl(Gamepad gmpdd){
        gmpd = gmpdd;
    }

    double startTime = elapsedTime.seconds();
    int rumbleCounter = 0;

    public void Run(){
        RumbleSequence(105, 110, 250);
        RumbleSequence(110, 120, 1000);
    }

    public void RumbleSequence(int startTiming, int endTiming, int duration){
        double currentTime = (int)(elapsedTime.seconds() - startTime);
        if(rumbleCounter<((int)(currentTime-startTiming)) && (int)currentTime <= endTiming){
            rumbleCounter = (int)currentTime-startTiming;
            gmpd.rumble(duration);
        }
    }
}
