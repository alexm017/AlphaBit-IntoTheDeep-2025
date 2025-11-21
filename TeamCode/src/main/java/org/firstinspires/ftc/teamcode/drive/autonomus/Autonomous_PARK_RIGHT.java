package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.structure.IntakeControl;

@Autonomous
public class Autonomous_PARK_RIGHT extends LinearOpMode {
    SampleMecanumDrive drive;
    IntakeControl intakeControl;

    TrajectorySequence trajectory1;

    Pose2d startPose = new Pose2d(11, -58.5, Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(11, -57))
                .lineTo(new Vector2d(56.5, -57))
                .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(trajectory1);

        while(opModeIsActive()){
            drive.update();
        }
    }
}
