package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.autoTests.yelllow;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.SubSystems.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Autonomous(name = "long Blue Park", group = "Auto")
public class longBluePark extends LinearOpMode {

    boolean tagFound = false;
    Robot robot;
    DistanceSensor distanceSensor, distanceSensor2, distanceSensor3;
    ElapsedTime timer = new ElapsedTime();
    Pose2d start = new Pose2d(0, 0, Math.toRadians(180));
    OpenCvCamera camera,camera2;
    yellowDetect yellowDetection;
    aprilTagDetection aprilTagDetectionPipeline;
    double aprilLoc;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    longBlueObjectDetect blueDetection;
    Pose2d currentPose;
    AprilTagDetection tagOfInterest = null;
    state currentState = state.IDLE;
    enum state{
        IDLE, truss1,board1, wall1, wall2,stack1, park, intakeStack
    }
    double distanceFront, distanceSide, distanceBack;
    ColorRangeSensor claw1,claw2,colorBoard;
    String followingPath;
    double claw1Val,claw2Val,colorBoardVal;
    int curCycle = 0;
    int boardColorThreshold = 600;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        initHardware();
        initColorDetection();
        waitForStart();
        camera.stopStreaming();

        if (isStopRequested()) return;
        telemetry.addLine(blueDetection.getLocation() + "");
        telemetry.update();
        //if (blueDetection.getLocation().equals("RIGHT"))
        if (false)
        {
            followingPath = "RIGHT";
            TrajectorySequence tape = drive.trajectorySequenceBuilder(start)
                    .addTemporalMarker(2,() -> {
                        robot.Claw.setTape();
                    })
                    .lineToConstantHeading(new Vector2d(40,-8))
                    .lineToConstantHeading(new Vector2d(53,-8))
                    .turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(54,65,Math.toRadians(-70)))
                    .build();
            aprilLoc = 29.5;
            drive.followTrajectorySequenceAsync(tape);

        }else if(true)
        {
            followingPath = "MIDDLE";
            TrajectorySequence tape = drive.trajectorySequenceBuilder(start)
                    .addTemporalMarker(1,() -> {
                        robot.Claw.setTape();
                    })
                    .lineToLinearHeading(new Pose2d(32,-3,Math.toRadians(0)))
                    .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(-90)))
                    .build();
            aprilLoc = 26.5;
            drive.followTrajectorySequenceAsync(tape);

        }//else if(blueDetection.getLocation().equals("LEFT")){
        else if (false){
            followingPath = "LEFT";
            TrajectorySequence tape = drive.trajectorySequenceBuilder(start)
                    .addTemporalMarker(2,() -> {
                        robot.Claw.setTape();
                    })
                    .lineToLinearHeading(new Pose2d(30,3, Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(30,9))
                    .lineToConstantHeading(new Vector2d(30,5))
                    .lineToLinearHeading(new Pose2d(53,-5,Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(54,65,Math.toRadians(-70)))
                    .build();
            aprilLoc = 22;
            drive.followTrajectorySequenceAsync(tape);
        }

        currentState = state.truss1;
        robot.Claw.setPosition(armState.close);
        robot.wrist.setPosition(armState.intakingCLAW);
        robot.Arm.intake();
        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.STATION);
        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case truss1:
                    if (distanceSide < 7 && distanceFront > 20){
                        Pose2d atWall = new Pose2d(distanceSide,distanceFront,Math.toRadians(-90));
                        drive.setPoseEstimate(atWall);
                        TrajectorySequence throughTruss = drive.trajectorySequenceBuilder(atWall)
                                        .addTemporalMarker(.1,() -> {
                                            robot.Arm.half();
                                        })
                                        .addTemporalMarker(2,()->{
                                            if (curCycle == 0){
                                                robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.AUTO1);
                                            }else{
                                                //robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.HIGHIN);
                                            }
                                        })
                                        .addTemporalMarker(3,() -> {
                                            if (curCycle == 0){
                                                robot.wrist.setPosition(armState.outtaking);
                                            }else{
                                                robot.wrist.autoOuttake();
                                            }
                                            robot.Claw.afterTape();
                                        })
                                        .addTemporalMarker(3.25,() -> {
                                            if (curCycle == 0){
                                                robot.Arm.setPosition(armState.outtaking);
                                            }else{
                                                robot.Arm.autoOuttake();
                                            }
                                        })
                                        .lineToLinearHeading(new Pose2d(5,75,Math.toRadians(-90)))
                                        .splineToConstantHeading(new Vector2d(29,105),Math.toRadians(280)).build();
                        drive.followTrajectorySequenceAsync(throughTruss);
                        currentState = state.board1;
                    }
                    break;
                case board1:
                    if (!drive.isBusy()){
                        Pose2d boardReallign = new Pose2d(distanceSide, 93 - distanceBack, Math.toRadians(-90));
                        drive.setPoseEstimate(boardReallign);

                        TrajectorySequence board1 = drive.trajectorySequenceBuilder(boardReallign)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(aprilLoc,94)).build();
                        drive.followTrajectorySequenceAsync(board1);
                        if (curCycle == 1){
                            currentState = state.park;
                        }else{
                            currentState = state.wall1;
                        }
                    }
                    break;
                case wall1:
                    if (colorBoardVal > boardColorThreshold || (curCycle == 1 && distanceBack < 7)){
                        robot.Claw.setPosition(armState.open);
                        TrajectorySequence stack1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(.5,() -> {
                                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TOPSTACK);
                                })
                                .addTemporalMarker(1,() -> {
                                    robot.wrist.topStack();
                                    robot.Arm.topStack();
                                })
                                .lineToLinearHeading(new Pose2d(8,62,Math.toRadians(-90))).build();
                        drive.followTrajectorySequenceAsync(stack1);
                        currentState = state.wall2;
                    }
                    break;
                case wall2:
                    if (!drive.isBusy()){
                        Pose2d wallReallign = new Pose2d(distanceSide,currentPose.getY(),Math.toRadians(-90));
                        drive.setPoseEstimate(wallReallign);
                        TrajectorySequence wall2 = drive.trajectorySequenceBuilder(wallReallign)
                                .lineToConstantHeading(new Vector2d(7,15))
                                .splineToConstantHeading(new Vector2d(31,-8),Math.toRadians(280)).build();
                        drive.followTrajectorySequenceAsync(wall2);
                        currentState = state.stack1;
                    }
                    break;
                case stack1:
                    if (distanceSide > 27 && distanceSide < 50 && distanceFront < 60){
                        Pose2d wallReallign = new Pose2d(distanceSide,distanceFront,Math.toRadians(-90));
                        drive.setPoseEstimate(wallReallign);
                        TrajectorySequence stack2 = drive.trajectorySequenceBuilder(wallReallign)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(13, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(27,0))
                                .build();
                        drive.followTrajectorySequenceAsync(stack2);
                        telemetry.addLine(drive.getPoseEstimate().getX() + "  " + drive.getPoseEstimate().getY());
                        currentState = state.intakeStack;
                    }
                    break;
                case intakeStack:
                    if (claw2Val > 1000 || claw1Val > 1000){
                        robot.Claw.setPosition(armState.close);
                        drive.setPoseEstimate(new Pose2d(distanceSide,distanceFront,Math.toRadians(-90)));
                        TrajectorySequence toStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(6,30,Math.toRadians(-90)))
                                .build();
                        drive.followTrajectorySequenceAsync(toStack);
                        currentState = state.truss1;
                        curCycle += 1;
                    }
                    break;
                case park:
                    if (!drive.isBusy()){
                        Pose2d boardReallign = new Pose2d(distanceSide,93-distanceBack,Math.toRadians(-90));
                        drive.setPoseEstimate(boardReallign);
                        robot.Arm.setPosition(armState.medium);
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.RESET);
                        robot.wrist.setPosition(armState.intakingCLAW);
                        TrajectorySequence park = drive.trajectorySequenceBuilder(boardReallign)
                                .addTemporalMarker(2,() -> {
                                    robot.Arm.setPosition(armState.low);
                                })
                                .lineToConstantHeading(new Vector2d(20,84))
                                .lineToLinearHeading(new Pose2d(10,84,Math.toRadians(0))).build();
                        drive.followTrajectorySequenceAsync(park);
                        currentState = state.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
            currentPose = drive.getPoseEstimate();
            distanceFront = distanceSensor.getDistance(DistanceUnit.INCH);
            distanceSide = distanceSensor2.getDistance(DistanceUnit.INCH);
            distanceBack = distanceSensor3.getDistance(DistanceUnit.INCH);
            claw1Val = claw1.blue()+claw1.red()+claw1.green();
            claw2Val = claw2.blue()+claw2.red()+claw2.green();
            colorBoardVal = colorBoard.red() + colorBoard.green() + colorBoard.blue();
            updateTelemetry();
            telemetry.update();
            drive.update();
        }
    }
    void detectTags() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if (currentDetections.size() != 0) {
            tagFound = false;
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == 2 || tag.id == 2 || tag.id == 2) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }
        }

    }
    private void initColorDetection() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        blueDetection = new longBlueObjectDetect(telemetry);

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
    void initAprilTagDetect(){
        aprilTagDetectionPipeline = new aprilTagDetection(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
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
    private void initYellowDetection() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        yellowDetection = new yellowDetect(telemetry);

        camera2.setPipeline(yellowDetection);
        camera2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera2.showFpsMeterOnViewport(true);
                camera2.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Unspecified Error Occurred; Camera Opening");
            }
        });
    }
    public void initHardware(){
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        distanceSensor3 = hardwareMap.get(DistanceSensor.class, "distanceSensor3");
        colorBoard = hardwareMap.get(ColorRangeSensor.class, "colorBoard");
        claw1 = hardwareMap.get(ColorRangeSensor.class, "claw1");
        claw2 = hardwareMap.get(ColorRangeSensor.class, "claw2");
    }
    private void updateTelemetry(){
        telemetry.addData("claw1 Value",  claw1Val);
        telemetry.addData("claw2 Value", claw2Val);
        telemetry.addData("color board value", colorBoardVal);
        telemetry.addData("side distance", distanceSide);
        telemetry.addData("front distance", distanceFront);
        telemetry.addData("back distance", distanceBack);
    }
}