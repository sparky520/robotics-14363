package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.autoExecutable.*;
import org.firstinspires.ftc.teamcode.drive.autoTests.shortBlueObjectDetectTest;
import org.firstinspires.ftc.teamcode.drive.autoTests.shortBlueTest;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.SubSystems.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "short Blue", group = "Auto")
public class shortBlue extends LinearOpMode {
    boolean tagFound = false;
    boolean caseTagFound = false;
    Robot robot;
    ElapsedTime timer = new ElapsedTime();
    Pose2d start = new Pose2d(0, 0, Math.toRadians(180));
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    double boardX, boardY;
    aprilTagDetection aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    shortBlueObjectDetectTest blueDetect;
    int location = 2;
    Pose2d currentPose;
    Pose2d boardPose = new Pose2d(26,43,Math.toRadians(-90));
    Pose2d stackPose = new Pose2d(50,-75,Math.toRadians(-90));
    AprilTagDetection tagOfInterest = null;
    state currentState = state.IDLE;

    enum state{
        IDLE, board1, stack1, checkStack1, board2, checkBoard2,
    }
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d newStart = new Pose2d();
        TrajectorySequence center = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToLinearHeading(new Pose2d(37,7,Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToConstantHeading(new Vector2d(36,15.01))
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                    robot.Arm.setPosition(armState.high);
                })
                .lineToConstantHeading(new Vector2d(31,15.02))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(24,34.5))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(26,24))
                //.waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                })
                .lineToConstantHeading(new Vector2d(0,29))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(0,36))
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToLinearHeading(new Pose2d(31,17.5,Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToConstantHeading(new Vector2d(31,17.51))
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                    robot.Arm.setPosition(armState.high);
                })
                .lineToConstantHeading(new Vector2d(31,17.52))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(22.5,36.75))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(20,24))
                //.waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                })
                .lineToConstantHeading(new Vector2d(0,29))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(0,36))
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToLinearHeading(new Pose2d(28,10,Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(28,-5))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToConstantHeading(new Vector2d(28,-4.99))
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                    robot.Arm.setPosition(armState.high);
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                //.lineToConstantHeading(new Vector2d(31,9.02))
                //.waitSeconds(1)
                .lineToConstantHeading(new Vector2d(33.75,36.75))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(33,24))
                //.waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                })
                .lineToConstantHeading(new Vector2d(0,23))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(0,33))
                .build();
        initCam();
        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;
        if (blueDetection.getLocation().equals("LEFT")){
            drive.followTrajectorySequence(left);
        }else if(blueDetection.getLocation().equals("MIDDLE")){
            drive.followTrajectorySequence(center);
        }else if(blueDetection.getLocation().equals("RIGHT")){
            drive.followTrajectorySequence(right);
        }
        else{
            telemetry.addLine("wrong" + blueDetection.getLocation());
        }
        telemetry.update();
    }
    private void initCam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = "Webcam 1";
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        blueDetection = new shortBlueObjectDetect(telemetry);
        camera.setPipeline(blueDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.showFpsMeterOnViewport(true);
                camera.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Unspecified Error Occurred; Camera Opening");
            }
        });
    }
    public void initCam2(){

    }
}