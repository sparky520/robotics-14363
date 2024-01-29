package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private GamepadEx driver, operator;
    boolean tagFound;
    int location = 9;
    AprilTagDetection tagOfInterest = null;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    private Robot robot;
    enum state{
        outtake,outtake2,outtake3, low,low2,low3,low4,IDLE
    }
    ElapsedTime timer = new ElapsedTime();
    state armPos = state.IDLE;
    SampleMecanumDrive drive;
    opAprilTagDetect aprilTag = new opAprilTagDetect(tagsize, fx, fy, cx, cy);
    OpenCvCamera camera;

    @Override
    public void init()
    {

        telemetry.setMsTransmissionInterval(50);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        initAprilTag();
    }
    @Override
    public void loop() {

        driver.readButtons();
        operator.readButtons();

        robot.drivetrain.fieldCentric(driver);

        switch (armPos){
            case outtake:
                timer.reset();
                robot.Arm.setPosition(armState.outtaking);
                armPos = state.outtake2;
                break;
            case outtake2:
                if (timer.seconds() > .3){
                    //robot.wrist.setPosition(armState.outtaking);
                    armPos = state.IDLE;
                }
                break;
            case low:
                robot.Arm.setPosition(armState.medium);
                //robot.wrist.setPosition(armState.intakingCLAW);
                armPos = state.low2;
                timer.reset();
                break;
            case low2:
                if (timer.seconds() > .4){
                    robot.Arm.setPosition(armState.low);
                    armPos = state.IDLE;
                }
                break;
        }
        if (gamepad2.square){
            armPos = state.low;
        }
        if (gamepad2.triangle){
         robot.Arm.setPosition(armState.medium);
        }
        if (gamepad2.circle){
            armPos = state.outtake;
        }

        if (gamepad1.left_bumper){
            robot.Claw.setPosition(armState.outtaking);
        }
        if (gamepad1.right_bumper){
            robot.Claw.setPosition(armState.intakingCLAW);
        }

        if (gamepad2.dpad_down){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
        }
        if (gamepad2.dpad_left){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.LOWIN);
        }
        if (gamepad2.dpad_up){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.MEDIUMIN);
        }
        if (gamepad2.dpad_right){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.HIGHIN);
        }

        if (gamepad1.triangle){
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {

                }
            });

        }
        telemetry.update();



    }
    public void initAprilTag(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }
    void detectTags() {
        ArrayList<AprilTagDetection> currentDetections = aprilTag.getLatestDetections();
        if (currentDetections.size() != 0) {
            tagFound = false;
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == location) {
                    telemetry.addLine("bobreg");
                    telemetry.update();
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }
        }
    }
}