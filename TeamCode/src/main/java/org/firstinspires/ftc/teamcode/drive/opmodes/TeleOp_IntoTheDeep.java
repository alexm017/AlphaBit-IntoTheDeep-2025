package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Testers.IntakeTesting;
import org.firstinspires.ftc.teamcode.drive.structure.ArmControl;
import org.firstinspires.ftc.teamcode.drive.structure.ChasisControl;
import org.firstinspires.ftc.teamcode.drive.structure.DriverAttentionControl;
import org.firstinspires.ftc.teamcode.drive.structure.TractionControlSystem;
import org.firstinspires.ftc.teamcode.drive.structure.IntakeControl;
import org.firstinspires.ftc.teamcode.drive.structure.SlidersControl;

import java.io.IOException;
import java.util.Arrays;

@TeleOp
public class TeleOp_IntoTheDeep extends LinearOpMode {

    MultipleTelemetry telemetrys;
    ChasisControl chasis_control;
    IntakeControl intakeControl;
    ArmControl armControl;
    SlidersControl slidersControl;

    double loopTime;
    double loop;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetrys = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armControl = new ArmControl(hardwareMap, gamepad2, gamepad1);
        slidersControl = new SlidersControl(hardwareMap, gamepad2, armControl);
        intakeControl = new IntakeControl(hardwareMap, gamepad2, slidersControl, armControl, gamepad1);
        slidersControl.intakeInit(intakeControl);
        armControl.slidersInit(slidersControl);
        armControl.intakeInit(intakeControl);
        chasis_control = new ChasisControl(hardwareMap, gamepad2);

        waitForStart();

        intakeControl.ClawControl(true);
        intakeControl.init();

        while(opModeIsActive()){
            chasis_control.Run(); // > ~10000
            intakeControl.Run(); // > ~1000
            armControl.Run(); // 120
            slidersControl.Run(); // 60
            telemetrys.addData("arm has reached ref ", armControl.hasReachedReference);
            telemetrys.addData("Detected Sample Angle ", intakeControl.sampleAngle);
            telemetrys.addData("Is Sample Pick-Up Automation Disabled ", intakeControl.disableAutomatedSamplePickUp);
            loop = System.nanoTime();
            telemetrys.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetrys.addData("left slider pos ", slidersControl.LeftSliderPos);
            telemetrys.addData("right slider pos ", slidersControl.RightSliderPos);
            telemetrys.addData("third slider pos ", slidersControl.thirdsliderpos);
            telemetrys.addData("sliders velocity ", slidersControl.SlidersVelocity);
            telemetrys.addData("Sliders busy status ", slidersControl.isBusy());
            telemetrys.addData("Sliders has reached reference ", slidersControl.isNigger);
            telemetrys.addData("sliders ref ", slidersControl.reference);
            telemetrys.addData("Sliders position 2 ", slidersControl.SlidersPosition);
            telemetrys.addData("Arm busy status ", armControl.isManualBusy());
            telemetrys.addData("Arm pos ", armControl.arm_position);
            telemetrys.addData("Arm reference ", armControl.reference);
            telemetrys.addData("Arm primary output ", armControl.getPrimaryOutput);
            telemetrys.addData("Arm primary derivative ", armControl.getDerivativeOutput);
            telemetrys.addData("Arm velocity ", armControl.arm_velocity);
            telemetrys.update();
        }
    }
}
