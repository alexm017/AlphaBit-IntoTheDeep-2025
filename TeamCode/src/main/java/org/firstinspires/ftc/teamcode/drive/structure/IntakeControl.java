package org.firstinspires.ftc.teamcode.drive.structure;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.AI_Development.BlobDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Config
public class IntakeControl {

    SlidersControl slidersControl;
    ArmControl armControl;
    ElapsedTime timer = new ElapsedTime();
    private OpenCvCamera webcam;
    private BlobDetectionPipeline pipeline;
    SampleMecanumDrive drive;

    Servo XServo;
    Servo YServo;
    Servo intakeArm;
    Servo intakeServo;
    Servo intakeArm2;

    Gamepad gamepad_servo;
    Gamepad second_gamepad;

    TrajectorySequence getSampleTraj;

    public IntakeControl(HardwareMap hwdmap, Gamepad gmpd, SlidersControl slidersControll, ArmControl armControll, Gamepad second_gmpd){
        armControl = armControll;
        slidersControl = slidersControll;
        XServo = hwdmap.get(Servo.class, "XServo");
        YServo = hwdmap.get(Servo.class, "YServo");
        intakeArm = hwdmap.get(Servo.class, "intakeArm");
        intakeServo = hwdmap.get(Servo.class, "intakeServo");
        intakeArm2 = hwdmap.get(Servo.class, "intakeArm2");

        drive = new SampleMecanumDrive(hwdmap);

        int cameraMonitorViewId = hwdmap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hwdmap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwdmap.get(WebcamName.class, "AICam"), cameraMonitorViewId);

        pipeline = new BlobDetectionPipeline();

        webcam.setPipeline(pipeline);

        FtcDashboard.getInstance().startCameraStream(webcam, 15);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        second_gamepad = second_gmpd;
        gamepad_servo = gmpd;
    }

    Pose2d startPose = new Pose2d(-37.5, -60 , Math.toRadians(90));
    double intakeArm_default = 0.55;
    double intakeArm_getSpecimen = 0.65;
    double intakeArm_putSpecimen = 0.48;
    double intakeArm_getSample = 0.475;
    double intakeArm_basket = 0.45;
    double intakeArm_straight = 0.48;
    double intakeArm_getSampleAutomatedTeleOp = 0.1525;
    double intakeArm_getSampleAutomatedAuto = 0.12;

    public double YServo_getSpecimen = 0.43;
    public double YServo_getSample = 0.58;
    double YServo_putSpecimen = 0.35;
    double YServo_default = 0.58;
    double YServo_basket = 0.105;
    double YServo_straight = 0.26;

    double XServo_Mid = 0.5;

    public static double intakeServo_openPOS = 1;
    public static double intakeServo_closePOS = 0;

    double xservo_temp_pos = XServo_Mid;
    boolean buttonPressed = false;
    boolean stopControl = false;
    boolean pickupTimer = false;
    boolean timerCheck = false;

    public double intakeYOffset = 0.00;
    boolean offsetCheck = false;
    public double sampleAngle = 0.00;
    boolean isMichaelJacksonAlive = false;
    boolean wantsToPickUpSampleAutomated = false;
    boolean needsToMove = false;
    boolean firstTime = false;
    boolean sFirstTime = false;
    boolean needsToMoveX = false;
    public boolean disableAutomatedSamplePickUp = false;

    public void init(){
        intakeArm.setPosition(intakeArm_default);
        intakeArm2.setPosition(intakeArm_default);
        YServo.setPosition(YServo_default);
        XServo.setPosition(XServo_Mid);

        drive.setPoseEstimate(startPose);
    }

    public void safeInit(){
        intakeArm.setPosition(intakeArm_default + 0.05);
        intakeArm2.setPosition(intakeArm_default + 0.05);
        YServo.setPosition(YServo_default);
        XServo.setPosition(XServo_Mid);

        drive.setPoseEstimate(startPose);
    }
    public double distanceToSample = 0.00;
    double savedSampleAngle = 0.00;
    public double distanceXSample = 0.00;

    public void Run(){
        boolean lockedControls = armControl.lock_intake_controls;
        sampleAngle = pipeline.getAngle();
        distanceToSample = pipeline.getDistanceToSample();
        distanceXSample = pipeline.getXDistanceToSample();

        if(gamepad_servo.left_trigger > 0.25){
            intakeServo.setPosition(intakeServo_openPOS);
        }else if(gamepad_servo.right_trigger > 0.25){
            intakeServo.setPosition(intakeServo_closePOS);
        }

        if(!lockedControls) {
            if (gamepad_servo.x) {
                xservo_temp_pos = xservo_temp_pos + 0.02;
                if (!buttonPressed) {
                    YServo.setPosition(YServo_default);
                    buttonPressed = true;
                }
                XServo.setPosition(xservo_temp_pos);
            } else if (gamepad_servo.b) {
                xservo_temp_pos = xservo_temp_pos - 0.02;
                if (!buttonPressed) {
                    YServo.setPosition(YServo_default);
                    buttonPressed = true;
                }
                XServo.setPosition(xservo_temp_pos);
            } else if (gamepad_servo.right_stick_button) {
                xservo_temp_pos = XServo_Mid;
                if (!buttonPressed) {
                    YServo.setPosition(YServo_default);
                    buttonPressed = true;
                }
                XServo.setPosition(xservo_temp_pos);
            } else {
                buttonPressed = false;
            }

            if (gamepad_servo.a) {
                intakeArm.setPosition(intakeArm_getSample);
                intakeArm2.setPosition(intakeArm_getSample);
                YServo.setPosition(YServo_getSample);
                pickupTimer = true;
                stopControl = false;
            }

            if (pickupTimer) {
                if (!timerCheck) {
                    timer.reset();
                    timerCheck = true;
                    intakeServo.setPosition(intakeServo_openPOS);
                }
                if(!stopControl && timer.milliseconds() > 250){
                    intakeServo.setPosition(intakeServo_closePOS);
                    stopControl = true;
                }
                if (timer.milliseconds() > 500) {
                    pickupTimer = false;
                    timerCheck = false;
                    stopControl = false;
                    firstTime = false;
                    sFirstTime = false;
                    intakeArm.setPosition(intakeArm_default);
                    intakeArm2.setPosition(intakeArm_default);
                    YServo.setPosition(YServo_default);
                    XServo.setPosition(XServo_Mid);
                    xservo_temp_pos = XServo_Mid;
                }
            }
        }
    }

    public void prepareIntake(int prepare){
        switch(prepare){
            case 0:
                intakeArm.setPosition(intakeArm_default);
                intakeArm2.setPosition(intakeArm_default);
                break;
            case 1:
                intakeArm.setPosition(intakeArm_getSpecimen);
                intakeArm2.setPosition(intakeArm_getSpecimen);
                break;
            case 2:
                intakeArm.setPosition(intakeArm_putSpecimen);
                intakeArm2.setPosition(intakeArm_putSpecimen);
                break;
            case 3:
                intakeArm.setPosition(intakeArm_getSample);
                intakeArm2.setPosition(intakeArm_getSample);
                break;
        }
    }

    public void prepareIntakeAutomated(boolean is_autonomous){
        if(!is_autonomous){
            intakeArm.setPosition(intakeArm_getSampleAutomatedTeleOp);
            intakeArm2.setPosition(intakeArm_getSampleAutomatedTeleOp);
        }else{
            intakeArm.setPosition(intakeArm_getSampleAutomatedAuto);
            intakeArm2.setPosition(intakeArm_getSampleAutomatedAuto);
        }
        YServo.setPosition(YServo_default);
        XServo.setPosition(XServo_Mid);
    }
    public void AxisControl(boolean prepare, boolean yservo_axis, boolean xservo_axis, double value){
        if(prepare){
            prepareIntake((int)value);
        }else if(yservo_axis){
            YServo.setPosition(value);
        }else if(xservo_axis){
            XServo.setPosition(value);
        }
    }

    public void ClawControl(boolean want_open){
        if(want_open){
            intakeServo.setPosition(intakeServo_openPOS);
        }else{
            intakeServo.setPosition(intakeServo_closePOS);
        }
    }
    public void getRotatedSample(){
        intakeArm.setPosition(intakeArm_getSample);
        intakeArm2.setPosition(intakeArm_getSample);
        YServo.setPosition(YServo_getSample);
    }

    public void getSample(){
        intakeArm.setPosition(intakeArm_getSample);
        intakeArm2.setPosition(intakeArm_getSample);
        YServo.setPosition(YServo_getSample);
        XServo.setPosition(XServo_Mid);
    }
    public void SampleClaw(double getit){
        if(getit == 0){
            intakeServo.setPosition(intakeServo_closePOS);
        }else if(getit == 1) {
            intakeServo.setPosition(intakeServo_openPOS);
        }
    }

    public void getSpecimen(boolean up_state){
        if(up_state){
            intakeArm.setPosition(intakeArm_getSpecimen);
            intakeArm2.setPosition(intakeArm_getSpecimen);
            YServo.setPosition(YServo_getSpecimen);
            XServo.setPosition(XServo_Mid);
        }else{
            intakeArm.setPosition(intakeArm_default);
            intakeArm2.setPosition(intakeArm_default);
            YServo.setPosition(YServo_default);
            XServo.setPosition(XServo_Mid);
        }
    }

    public void placeSpecimen(boolean to_place){
        if(to_place){
            intakeArm.setPosition(intakeArm_putSpecimen);
            intakeArm2.setPosition(intakeArm_putSpecimen);
            YServo.setPosition(YServo_putSpecimen);
            XServo.setPosition(XServo_Mid);
        }else {
            intakeArm.setPosition(intakeArm_default);
            YServo.setPosition(YServo_default);
            XServo.setPosition(XServo_Mid);
        }
    }

    public void scoreBasketSample(boolean score_basket){
        if(score_basket) {
            intakeArm.setPosition(intakeArm_basket);
            intakeArm2.setPosition(intakeArm_basket);
            YServo.setPosition(YServo_basket);
            XServo.setPosition(XServo_Mid);
        }else{
            intakeArm.setPosition(intakeArm_default);
            intakeArm2.setPosition(intakeArm_default);
            YServo.setPosition(YServo_default);
            XServo.setPosition(XServo_Mid);
        }
    }

    public void straightBasket() {
        intakeArm.setPosition(intakeArm_straight);
        intakeArm2.setPosition(intakeArm_straight);
        YServo.setPosition(YServo_straight);
        XServo.setPosition(XServo_Mid);
    }
}
