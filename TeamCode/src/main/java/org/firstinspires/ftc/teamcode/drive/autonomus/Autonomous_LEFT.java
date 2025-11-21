package org.firstinspires.ftc.teamcode.drive.autonomus;

import static org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.structure.ArmControl;
import org.firstinspires.ftc.teamcode.drive.structure.IntakeControl;
import org.firstinspires.ftc.teamcode.drive.structure.SlidersControl;

@Autonomous
public class Autonomous_LEFT extends LinearOpMode {
    SampleMecanumDrive drive;
    IntakeControl intakeControl;
    ArmControl armControl;
    SlidersControl slidersControl;

    TrajectorySequence trajectory;

    Pose2d startPose = new Pose2d(-37.5, -60 , Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        armControl = new ArmControl(hardwareMap, gamepad2, gamepad1);
        slidersControl = new SlidersControl(hardwareMap, gamepad2, armControl);
        intakeControl = new IntakeControl(hardwareMap, gamepad2, slidersControl, armControl, gamepad1);
        slidersControl.intakeInit(intakeControl);
        armControl.slidersInit(slidersControl);
        armControl.intakeInit(intakeControl);

        drive.setPoseEstimate(startPose);

        trajectory = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> armControl.SetToReference(870))
                .addTemporalMarker(() -> intakeControl.straightBasket())
                .waitSeconds(0.25)
                .addTemporalMarker(() -> slidersControl.SetToReference(2350))
                .lineToLinearHeading(new Pose2d(-53.65,-50.775, Math.toRadians(45)))
                .waitSeconds(12.5)
                .addTemporalMarker(() -> intakeControl.scoreBasketSample(true))
                .waitSeconds(2)
                .addTemporalMarker(() -> intakeControl.ClawControl(true))
                .waitSeconds(2)
                .addTemporalMarker(() -> intakeControl.ClawControl(false))
                .addTemporalMarker(() -> intakeControl.straightBasket())
                .waitSeconds(2)
                .addTemporalMarker(() -> slidersControl.SetToReference(0))
                .waitSeconds(12.5)
                .addTemporalMarker(() -> armControl.SetToReference(0))
                .addTemporalMarker(() -> intakeControl.init())
                .waitSeconds(5)
                .lineToLinearHeading(new Pose2d(-59.75 , -33.45, Math.toRadians(90)))
                .addTemporalMarker(() -> intakeControl.getSample())
                .waitSeconds(5)
                .addTemporalMarker(() -> intakeControl.init())
                .addTemporalMarker(() -> armControl.SetToReference(870))
                .lineToLinearHeading(new Pose2d(-53.55,-50.7, Math.toRadians(45)))
                .waitSeconds(10)
                .addTemporalMarker(() -> slidersControl.SetToReference(2350))
                .waitSeconds(10)
                .addTemporalMarker(() -> intakeControl.scoreBasketSample(true))
                .waitSeconds(2)
                .addTemporalMarker(() -> intakeControl.ClawControl(true))
                .waitSeconds(2)
                .addTemporalMarker(() -> intakeControl.ClawControl(false))
                .addTemporalMarker(() -> intakeControl.straightBasket())
                .waitSeconds(2)
                .addTemporalMarker(() -> slidersControl.SetToReference(0))
                .waitSeconds(12.5)
                .addTemporalMarker(() -> armControl.SetToReference(0))
                .addTemporalMarker(() -> intakeControl.init())
                .waitSeconds(5)




                /*.addTemporalMarker(() -> slidersControl.SetToReference(0))
                .waitSeconds(12.5)
                .waitSeconds(0.8)
                .addTemporalMarker(() -> armControl.SetToReference(0))
                .waitSeconds(12.5)
                .addTemporalMarker(() -> intakeControl.init())*/

                /*.lineToLinearHeading(new Pose2d(-47.9, -32.9, Math.toRadians(91)))
                .addTemporalMarker(() -> intakeControl.getSample())

                .waitSeconds(0.4)

                .addTemporalMarker(() -> armControl.SetToReference(1070))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> intakeControl.straightBasket())
                .lineToLinearHeading(new Pose2d(-53.55,-50.7, Math.toRadians(45)))
                .addTemporalMarker(() -> slidersControl.SetToReference(2000))
                .waitSeconds(1)
                .addTemporalMarker(() -> intakeControl.scoreBasketSample(true))
                .waitSeconds(0.2)

                .waitSeconds(0.2)

                .addTemporalMarker(() -> intakeControl.straightBasket())
                .waitSeconds(0.1)
                .addTemporalMarker(() -> slidersControl.SetToReference(0))
                .waitSeconds(0.8)
                .addTemporalMarker(() -> armControl.SetToReference(0))
                .addTemporalMarker(() -> intakeControl.init())
                .lineToLinearHeading(new Pose2d(-59.75 , -33.45, Math.toRadians(90)))
                .addTemporalMarker(() -> intakeControl.getSample())

                .waitSeconds(0.15)

                .addTemporalMarker(() -> armControl.SetToReference(1070))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> intakeControl.straightBasket())
                .lineToLinearHeading(new Pose2d(-53.55,-50.7, Math.toRadians(45)))
                .addTemporalMarker(() -> slidersControl.SetToReference(2000))
                .waitSeconds(1)
                .addTemporalMarker(() -> intakeControl.scoreBasketSample(true))
                .waitSeconds(0.2)

                .waitSeconds(0.2)

                .addTemporalMarker(() -> intakeControl.straightBasket())
                .waitSeconds(0.2)
                .addTemporalMarker(() -> slidersControl.SetToReference(0))
                .waitSeconds(1)
                .addTemporalMarker(() -> armControl.SetToReference(0))
                .addTemporalMarker(() -> intakeControl.init())
                .lineToLinearHeading(new Pose2d(-53.3, -42.85, Math.toRadians(135)))
                .addTemporalMarker(() -> intakeControl.AxisControl(false, false, true, 0.68))
                .addTemporalMarker(() -> slidersControl.SetToReference(1250))
                .waitSeconds(0.9)
                .addTemporalMarker(() -> intakeControl.getRotatedSample())

                .waitSeconds(0.25)

                .addTemporalMarker(() -> slidersControl.SetToReference(0))
                .addTemporalMarker(() -> intakeControl.init())
                .lineToLinearHeading(new Pose2d(-53.65,-50.93, Math.toRadians(43)))
                .addTemporalMarker(() -> intakeControl.straightBasket())
                .addTemporalMarker(() -> armControl.SetToReference(1070))
                .waitSeconds(1)
                .addTemporalMarker(() -> slidersControl.SetToReference(2000))
                .waitSeconds(1)
                .addTemporalMarker(() -> intakeControl.scoreBasketSample(true))
                .waitSeconds(0.2)

                .waitSeconds(0.2)

                .addTemporalMarker(() -> intakeControl.straightBasket())
                .waitSeconds(0.1)
                .addTemporalMarker(() -> slidersControl.SetToReference(0))
                .waitSeconds(0.8)
                .addTemporalMarker(() -> armControl.SetToReference(0))
                .addTemporalMarker(() -> intakeControl.init())*/
                .build();

        armControl.SetToReference(870);

        waitForStart();

        drive.followTrajectorySequenceAsync(trajectory);

        while(opModeIsActive()){
            drive.update();
            armControl.AutonomousUpdate();
            slidersControl.AutonomousUpdate();
            telemetry.addData("Sliders reference ", slidersControl.reference);
            telemetry.addData("Sliders position ", slidersControl.SlidersPosition);
            telemetry.addData("Sliders velocity ", slidersControl.SlidersVelocity);
            telemetry.addData("Arm reference ", armControl.reference);
            telemetry.addData("Arm position ", armControl.arm_position);
            telemetry.addData("Arm velocity ", armControl.arm_velocity);
            telemetry.update();
        }
    }
}
