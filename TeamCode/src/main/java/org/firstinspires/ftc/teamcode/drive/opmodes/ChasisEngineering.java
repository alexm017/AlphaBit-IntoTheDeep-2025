package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Testers.ArmPositionTesting;
import org.firstinspires.ftc.teamcode.drive.Testers.IntakeTesting;
import org.firstinspires.ftc.teamcode.drive.Testers.SlidersPositionTesting;
import org.firstinspires.ftc.teamcode.drive.structure.ChasisControl;

@TeleOp
public class ChasisEngineering extends LinearOpMode {

    ChasisControl chasis_control;
    MultipleTelemetry telemetrys;
    //SlidersPositionTesting slidersPositionTesting;
    //ArmPositionTesting armPositionTesting;
    IntakeTesting intakeTesting;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetrys = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        chasis_control = new ChasisControl(hardwareMap, gamepad1);
        //slidersPositionTesting = new SlidersPositionTesting(hardwareMap, gamepad1);
        //armPositionTesting = new ArmPositionTesting(hardwareMap, gamepad1);
        intakeTesting = new IntakeTesting(hardwareMap, gamepad2);

        waitForStart();

        while(opModeIsActive()){
            chasis_control.Run();
            //slidersPositionTesting.Run();
            //armPositionTesting.Run();
            intakeTesting.Run();
            //telemetrys.addData("left slider pos: ", slidersPositionTesting.leftSliderPos);
            //telemetrys.addData("right slider pos: ", slidersPositionTesting.rightSliderPos);
            //telemetrys.addData("arm pos: ", armPositionTesting.armpos);
            telemetry.addData("max pos", intakeTesting.max_position);
            telemetry.addData("min pos", intakeTesting.min_position);
//            telemetrys.addData("y servo pos ", intakeTesting.ypos);
//            telemetrys.addData("x servo pos ", intakeTesting.xpos);
            telemetry.update();
        }
    }
}
