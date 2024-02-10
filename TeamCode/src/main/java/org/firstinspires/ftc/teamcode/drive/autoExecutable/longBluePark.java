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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
    boolean caseTagFound = false;
    Robot robot;
    Pose2d facingBoard;
    DistanceSensor distanceSensor, distanceSensor2;
    ElapsedTime intake = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    Pose2d start = new Pose2d(0, 0, Math.toRadians(180));
    SampleMecanumDrive drive;
    Pose2d stack = new Pose2d(46,-10,Math.toRadians(-90));
    OpenCvCamera camera,camera2;
    yellowDetect yellowDetection;
    double boardX, boardY, stack1Y, stackDetectX,stackDetectY;
    aprilTagDetection aprilTagDetectionPipeline;
    final double tag1X = 22;
    final double tag2LEFTX = 26.5;
    final double tag2RIGHTX = 29.5;
    double aprilLoc;
    final double tag3X = 35;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    longBlueObjectDetect blueDetection;
    int location = 1;
    Pose2d currentPose;
    Pose2d boardPose;
    Pose2d stackPose = new Pose2d(50,-75,Math.toRadians(-90));
    AprilTagDetection tagOfInterest = null;
    state currentState = state.IDLE;
    double boardXOffset, pathXoffset;
    double boardYOffset = 0;
    boolean pixelDropped = false;
    boolean intaking = false;
    enum state{
        IDLE, truss1,board1, wall1, stack1, park, checkStack1, intakeStack, board2, checkBoard2,test
    }
    AprilTagDetection lastTOI = null;
    boolean armRaised = false;
    double distance, distance2;
    boolean strafeChecking = false;
    double initialDistance, y;
    Pose2d stackFound, strafePose;
    double totalChecks = 0;
    double totalDistance = 0;
    boolean firstStackFound = false;
    String followingPath;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        TrajectorySequence tape = drive.trajectorySequenceBuilder(start).lineToConstantHeading(new Vector2d(0,1.1)).build();
        TrajectorySequence board1 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,1)).build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(board1.end()).lineToConstantHeading(new Vector2d(0,1.2)).build();
        TrajectorySequence stack1 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,1.5)).build();
        TrajectorySequence checkStack1 = drive.trajectorySequenceBuilder(new Pose2d(0,1.6)).lineToConstantHeading(new Vector2d(0,2)).build();
        TrajectorySequence board2 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,2.5)).build();
        TrajectorySequence checkBoard2 = drive.trajectorySequenceBuilder(checkStack1.end()).lineToConstantHeading(new Vector2d(0,3)).build();
        TrajectorySequence t = drive.trajectorySequenceBuilder(checkStack1.end()).setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(0,13)).build();
        initColorDetection();
        waitForStart();
        camera.stopStreaming();
        initAprilTagDetect();

        if (isStopRequested()) return;
        telemetry.addLine(blueDetection.getLocation() + "");
        telemetry.update();
        //if (blueDetection.getLocation().equals("RIGHT"))
        if (false)
        {
            followingPath = "RIGHT";
            tape = drive.trajectorySequenceBuilder(start)
                    .addTemporalMarker(2,() -> {
                        robot.Claw.setTape();
                    })
                    .lineToConstantHeading(new Vector2d(40,-8))
                    .lineToConstantHeading(new Vector2d(53,-8))
                    .turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(54,65,Math.toRadians(-70)))
                    .build();
            boardXOffset = -28.5;
            boardYOffset = -6.5;
            aprilLoc = 29.5;
            boardPose = new Pose2d(23,43,Math.toRadians(90));

        }else if(true)
        {
            followingPath = "MIDDLE";
            tape = drive.trajectorySequenceBuilder(start)
                    .addTemporalMarker(1,() -> {
                        robot.Claw.setTape();
                    })
                    .lineToLinearHeading(new Pose2d(30,-3,Math.toRadians(0)))
                    .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(-90)))
                    .build();
            boardXOffset = -28.5;
            boardYOffset = -6.5;
            aprilLoc = 26.5;
            boardPose = new Pose2d(26,43,Math.toRadians(-90));

        }else if(blueDetection.getLocation().equals("LEFT")){
            followingPath = "LEFT";
            tape = drive.trajectorySequenceBuilder(start)
                    .addTemporalMarker(2,() -> {
                        robot.Claw.setTape();
                    })
                    .lineToLinearHeading(new Pose2d(30,3, Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(30,9))
                    .lineToConstantHeading(new Vector2d(30,5))
                    .lineToLinearHeading(new Pose2d(53,-5,Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(54,65,Math.toRadians(-70)))
                    .build();
            boardXOffset = -30;
            boardYOffset = -6;
            aprilLoc = 22;
            boardPose = new Pose2d(29,43,Math.toRadians(-90));
        }

        currentState = state.truss1;
        robot.Claw.setPosition(armState.close);
        robot.wrist.setPosition(armState.intakingCLAW);
        drive.followTrajectorySequenceAsync(tape);
        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case truss1:
                    if (distance2 < 25 && !drive.isBusy()){
                        Pose2d atWall = new Pose2d(distance2,0,Math.toRadians(-90));
                        drive.setPoseEstimate(atWall);
                        TrajectorySequence throughTruss = drive.trajectorySequenceBuilder(atWall)
                                        .addTemporalMarker(2,() -> {
                                            robot.Arm.setPosition(armState.outtaking);
                                        })
                                        .addTemporalMarker(4,() -> {
                                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.AUTO1);
                                            robot.wrist.setPosition(armState.outtaking);
                                            robot.Claw.afterTape();
                                        })
                                        .lineToConstantHeading(new Vector2d(5,75))
                                        .lineToConstantHeading(new Vector2d(32,77)).build();
                        drive.followTrajectorySequenceAsync(throughTruss);
                        currentState = state.board1;
                    }
                    break;
                case board1:
                    if (tagOfInterest != null && caseTagFound == false){
                        double distanceToBoard = 100*tagOfInterest.pose.z/6;
                        facingBoard = new Pose2d(distance2,93 - distanceToBoard,Math.toRadians(-90));
                        drive.setPoseEstimate(facingBoard);
                        board1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(aprilLoc,87.5))
                                .build();
                        caseTagFound = true;
                    }
                    if (!drive.isBusy()){
                        if (/*followingPath.equals("MIDDLE")*/false){
                            camera.closeCameraDevice();
                            initYellowDetection();
                            if (yellowDetection.getLocation().equals("RIGHT")){
                                board1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                        .lineToConstantHeading(new Vector2d(aprilLoc - 1.5,87.5))
                                        .build();
                            }
                            drive.followTrajectorySequenceAsync(board1);
                            currentState = state.IDLE;
                        }
                        else{
                            drive.followTrajectorySequenceAsync(board1);
                            currentState = state.wall1;
                        }
                    }
                    break;
                case wall1:
                    if (!drive.isBusy()){
                        robot.Claw.dropBoard();
                        stack1 = drive.trajectorySequenceBuilder(new Pose2d(aprilLoc,85,Math.toRadians(-90)))
                                .addTemporalMarker(.5,() -> {
                                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TOPSTACK);
                                })
                                .addTemporalMarker(1,() -> {
                                    robot.wrist.setPosition(armState.intakingCLAW);
                                    robot.Arm.topStack();
                                    robot.Claw.maxClose();
                                })
                                .waitSeconds(.2)
                                .lineToConstantHeading(new Vector2d(5,83))
                                .lineToConstantHeading(new Vector2d(5,15)).build();
                        drive.followTrajectorySequenceAsync(stack1);
                        currentState = state.stack1;
                        timer.reset();
                    }
                    break;
                case stack1:
                    if (!drive.isBusy() && distance < 60){
                        Pose2d wallReallign = new Pose2d(distance2 - 2,distance - stack.getY(),Math.toRadians(-90));
                        drive.setPoseEstimate(wallReallign);
                        TrajectorySequence stack2 = drive.trajectorySequenceBuilder(wallReallign)
                                        .lineToLinearHeading(stack).build();
                        telemetry.addLine(drive.getPoseEstimate().getX() + "    " +  drive.getPoseEstimate().getY());
                        drive.followTrajectorySequenceAsync(stack2);
                        currentState = state.intakeStack;
                    }
                    break;
                case intakeStack:
                    if (!drive.isBusy() && distance < 100){
                        TrajectorySequence toStack = drive.trajectorySequenceBuilder(currentPose)
                                .lineToConstantHeading(new Vector2d(currentPose.getX(),currentPose.getY() - distance))
                                .addDisplacementMarker(() -> {
                                    robot.Claw.setPosition(armState.close);
                                })
                                .lineToConstantHeading(new Vector2d(5,-5))
                                .build();
                        drive.followTrajectorySequenceAsync(toStack);
                        currentState = state.truss1;
                    }
                    break;
                case park:
                    if (!drive.isBusy()){
                        robot.Claw.dropBoard();
                        park = drive.trajectorySequenceBuilder(currentPose)
                                .addTemporalMarker(10,()->{
                                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.STATION);
                                    robot.Arm.setPosition(armState.medium);
                                    robot.wrist.setPosition(armState.intakingCLAW);
                                    robot.Claw.setPosition(armState.close);
                                })
                                .addTemporalMarker(14,()->{
                                    robot.Arm.setPosition(armState.low);
                                })
                                .forward(5)
                                .strafeRight(13)
                                .turn(Math.toRadians(90))
                                .build();
                        drive.followTrajectorySequenceAsync(park);

                        currentState = state.IDLE;
                    }
                case IDLE:
                    telemetry.addLine(distance2 + "");

            }

            currentPose = drive.getPoseEstimate();
            distance = distanceSensor.getDistance(DistanceUnit.INCH);
            distance2 = distanceSensor2.getDistance(DistanceUnit.INCH);
            telemetry.update();
            drive.update();
            detectTags();
        }
    }
    void detectTags() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if (currentDetections.size() != 0) {
            tagFound = false;
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                    tagOfInterest = tag;
//                    telemetry.addData("ID", tagOfInterest.id);
//                    telemetry.addData("X", 100*tagOfInterest.pose.x/6/1.41);
//                    telemetry.addData("Z", 100*tagOfInterest.pose.z/6);
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
}