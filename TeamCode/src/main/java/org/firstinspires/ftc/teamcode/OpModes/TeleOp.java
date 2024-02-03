package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.opAprilTagDetect;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.ArrayList;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    boolean rotated = false;
    private GamepadEx driver, operator;
    Pose2d currentPos,intakePos;
    boolean tagFound;
    int location = 9;
    AprilTagDetection tagOfInterest = null;
    opAprilTagDetect aprilTagDetect;
    DistanceSensor distanceSensor,distanceSensor2;
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
    double intakeX,intakeY,intakeRot;
    SampleMecanumDrive drive;
    opAprilTagDetect aprilTag = new opAprilTagDetect(tagsize, fx, fy, cx, cy);
    OpenCvCamera camera;
    double d,d2;
    double offset = 0;
    boolean autoIntake = false;
    @Override
    public void init()
    {
        timer.reset();
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        telemetry.setMsTransmissionInterval(50);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
        initAprilTag();
        timer.reset();
        robot.wrist.setPosition(armState.intakingCLAW);
        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TELEOPSTATION);
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
    @Override
    public void loop() {
        driver.readButtons();
        operator.readButtons();
        robot.drivetrain.fieldCentric(driver);
        d = distanceSensor.getDistance(DistanceUnit.INCH );
        d2 = distanceSensor2.getDistance(DistanceUnit.INCH );
        telemetry.addData("d1", d);
        telemetry.addData("d2", d2);

        if (gamepad1.triangle){
            robot.drivetrain.resetIMU();
        }
        if (gamepad1.right_trigger > 0){
            robot.drivetrain.slow_mode = .15;
        }else if (gamepad1.left_trigger > 0){
            robot.drivetrain.slow_mode = .3;
        }
        else{
            robot.drivetrain.slow_mode = 1;
        }

        if (gamepad2.circle){
            robot.Arm.setPosition(armState.outtaking);
            robot.wrist.setPosition(armState.outtaking);
        }
        if (gamepad1.square){
            robot.Airplane.setPosition(armState.airplaneLaunch);
        }
        if (gamepad2.triangle){
            robot.Arm.setPosition(armState.medium);
            robot.wrist.setPosition(armState.intakingCLAW);
        }
        if (gamepad2.square){
            robot.Arm.setPosition(armState.low);
            robot.wrist.setPosition(armState.intakingCLAW);
        }
        if (gamepad1.left_bumper){
            robot.Claw.setPosition(armState.open);
        }
        if (gamepad2.dpad_up){

            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.MEDIUMIN);
        }
        if (gamepad2.dpad_left){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.LOWIN);
        }
        if (gamepad2.dpad_right){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.HIGHIN);
        }

        if (gamepad2.dpad_down){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TELEOPSTATION);
        }
        if (gamepad1.right_bumper){
            robot.Claw.setPosition(armState.close);
        }
        if (gamepad1.triangle){
            robot.drivetrain.slow_mode = 1;
        }
        currentPos = drive.getPoseEstimate();
        telemetry.update();



    }
    void detectTags() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetect.getLatestDetections();
        if (currentDetections.size() != 0) {
            tagFound = false;
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == 1||tag.id == 2||tag.id == 3||tag.id == 4||tag.id == 5||tag.id == 6) {
                    tagOfInterest = tag;
                    double power = 1 - 8/tagOfInterest.pose.z/6;
                    if (power > .2){
                        robot.drivetrain.slow_mode = power;
                    }
                    telemetry.addData("slowmode", robot.drivetrain.slow_mode);
                    telemetry.addData("power", power);

                    gamepad1.rumble(1);
                    tagFound = true;
                    break;
                }
            }
        }
        else{
            telemetry.addLine("NOT FOUND");
        }
    }
    public void initAprilTag(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetect = new opAprilTagDetect(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetect);
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
        telemetry.setMsTransmissionInterval(50);
    }
}