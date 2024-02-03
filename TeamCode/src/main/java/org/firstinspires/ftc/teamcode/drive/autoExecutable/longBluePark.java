package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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
    DistanceSensor distanceSensor, distanceSensor2;
    ElapsedTime intake = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    Pose2d start = new Pose2d(0, 0, Math.toRadians(180));
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    double boardX, boardY, stack1Y, stackDetectX,stackDetectY;
    aprilTagDetection aprilTagDetectionPipeline;
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
        IDLE, board1, stack1, park, checkStack1, intakeStack, board2, checkBoard2,test
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
        if (blueDetection.getLocation().equals("RIGHT"))
        {
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
            boardPose = new Pose2d(23,43,Math.toRadians(90));

        }else if(blueDetection.getLocation().equals("MIDDLE"))
        {
            tape = drive.trajectorySequenceBuilder(start)
                    .addTemporalMarker(2,() -> {
                        robot.Claw.setTape();
                    })
                    .lineToConstantHeading(new Vector2d(46,3))
                    .lineToConstantHeading(new Vector2d(53,3))
                    .turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(53,65,Math.toRadians(-70)))
                    .build();
            boardXOffset = -28.5;
            boardYOffset = -6.5;
            boardPose = new Pose2d(26,43,Math.toRadians(90));

        }else if(blueDetection.getLocation().equals("LEFT")){
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
            boardPose = new Pose2d(29,43,Math.toRadians(-90));
        }

        currentState = state.board1;
        robot.Claw.setPosition(armState.close);
        robot.wrist.setPosition(armState.intakingCLAW);
        drive.followTrajectorySequenceAsync(tape);
        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case board1:
                    if (tagOfInterest != null && caseTagFound == false){
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX()+ boardXOffset - (100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY() + boardYOffset +(100*tagOfInterest.pose.z/6);
                        board1 = drive.trajectorySequenceBuilder(tape.end())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToLinearHeading(new Pose2d(boardX,boardY,Math.toRadians(-90)))
                                .build();
                        caseTagFound = true;
                    }
                    if (!drive.isBusy()){
                        robot.Claw.setTape();
                    }
                    if (!drive.isBusy() && caseTagFound == true){
                        robot.Claw.setTape();
                        outtake();
                        timer.reset();
                        armRaised = true;
                        location = 3;
                        drive.followTrajectorySequenceAsync(board1);
                    }
                    if (armRaised && timer.seconds() > 1){
                        caseTagFound = false;
                        currentState = state.park;
                        tagOfInterest = null;
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.AUTO1);
                    }
                case stack1:
                    if (tagOfInterest != null){
                        lastTOI = tagOfInterest;
                    }
                    if (!drive.isBusy()){
                        robot.Claw.dropBoard();
                        pixelDropped = true;
                    }
                    if (!drive.isBusy() && pixelDropped){
                        pixelDropped = false;
                        if (lastTOI.id == 3){
                            boardPose = new Pose2d(42 - (100*lastTOI.pose.x/6/1.41) ,43,Math.toRadians(-90));
                        }else if (lastTOI.id == 2){
                            boardPose = new Pose2d(36 - (100*lastTOI.pose.x/6/1.41) ,43,Math.toRadians(-90));
                        }else if (lastTOI.id == 1){
                            boardPose = new Pose2d(30 - (100*lastTOI.pose.x/6/1.41) ,43,Math.toRadians(-90));
                        }else{
                            telemetry.addLine("tag not found once more!");
                        }
                        drive.setPoseEstimate(boardPose);
                        stack1 = drive.trajectorySequenceBuilder(boardPose)
                                .addTemporalMarker(.5,() -> {
                                    robot.wrist.setPosition(armState.intakingCLAW);
                                    robot.Arm.topStack();
                                    robot.Claw.maxClose();
                                })
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .waitSeconds(.2)
                                .splineToConstantHeading(new Vector2d(53 + pathXoffset,22), Math.toRadians(280))
                                .addDisplacementMarker(() -> {
                                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TOPSTACK);
                                })
                                .splineToConstantHeading(new Vector2d(50+ pathXoffset,-25),Math.toRadians(280))
                                .build();
                        drive.followTrajectorySequenceAsync(stack1);
                        currentState = state.checkStack1;
                        timer.reset();
                    }
                    break;
                case checkStack1:
                    if (!drive.isBusy() && !strafeChecking){
                        y = currentPose.getY() - (distance2) + 9;
                        telemetry.addData("to wall", y);
                        TrajectorySequence lignUp = drive.trajectorySequenceBuilder(currentPose)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToLinearHeading(new Pose2d(50+ pathXoffset,y*.7,Math.toRadians(-90)))

                                .lineToConstantHeading(new Vector2d(50+ pathXoffset,y))
                                .addDisplacementMarker(()-> {
                                    strafeChecking = true;
                                    strafePose = drive.getPoseEstimate();
                                    robot.Claw.stack();

                                })
                                .lineToConstantHeading(new Vector2d(35,y))
                                .build();
                        drive.followTrajectorySequenceAsync(lignUp);
                    }
                    if (distance +distance2 > 200){
                        telemetry.addLine("Sensors not reading");
                        break;
                    }
                    else {
                        if (strafeChecking){
                            if (distance2 - distance  > -.7){
                                telemetry.addLine("Pixel found? " + distance);
                                telemetry.addData("difference", distance2 - distance);
                                telemetry.addData("distance",distance);
                                telemetry.addData("distance2",distance2);
                                stackFound = currentPose;
                                totalChecks += 1;
                                totalDistance += stackFound.getX() - strafePose.getX();
                                firstStackFound = true;
                            }
                            else{
                                telemetry.addLine("starin at wall  "+ distance);
                                telemetry.addLine(distance2 - distance  + "");
                                telemetry.addLine(distance+ "");
                                telemetry.addLine(distance2+ "");
                                if (firstStackFound){
                                    if (totalChecks > 1){
                                        strafeChecking = false;
                                        double stackDetectY = stackFound.getY() - distance2 - 1.5;
                                        double averageDistance = strafePose.getX() + (totalDistance/totalChecks) + 7.75;
                                        telemetry.addData("checks",totalChecks);
                                        telemetry.addData("avgD",averageDistance);
                                        telemetry.addData("avgD",averageDistance);
                                        t = drive.trajectorySequenceBuilder(currentPose)
                                                .lineToConstantHeading(new Vector2d(averageDistance,y))
                                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))

                                                .lineToConstantHeading(new Vector2d(averageDistance,stackDetectY))
                                                .addDisplacementMarker(()->{
                                                    robot.Claw.setPosition(armState.close);
                                                }).build();
                                        stackFound = null;
                                        currentState = state.intakeStack;
                                    }else{
                                        telemetry.addData("Checks", totalChecks);
                                    }

                                }
                            }
                        }
                    }
                    break;
                case intakeStack:
                    drive.followTrajectorySequenceAsync(t);
                    currentState = state.board2;
                case board2:
                    if (!drive.isBusy()){
                        drive.setPoseEstimate(new Pose2d(55,0,Math.toRadians(-90)));
                        board2 = drive.trajectorySequenceBuilder(new Pose2d(55,0,Math.toRadians(-90)))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))

                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(55,65)).build();
                        drive.followTrajectorySequenceAsync(board2);
                        location = 3;
                        currentState = state.checkBoard2;
                    }
                    break;
                case checkBoard2:
                    if (tagOfInterest != null){
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX() - 32 - (100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY() - 3 +(100*tagOfInterest.pose.z/6);
                        checkBoard2 = drive.trajectorySequenceBuilder(currentPose)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(boardX,boardY))
                                .build();
                        caseTagFound = true;
                    }
                    if (!drive.isBusy()){
                        robot.Arm.setPosition(armState.outtaking);
                        robot.wrist.setPosition(armState.outtaking);
                        location = 3;
                        drive.followTrajectorySequenceAsync(checkBoard2);
                        caseTagFound = false;
                        currentState = state.stack1;
                        tagOfInterest = null;
                        boardPose = new Pose2d(30,43,Math.toRadians(-90));
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

            }

            currentPose = drive.getPoseEstimate();
            distance = distanceSensor.getDistance(DistanceUnit.INCH);
            distance2 = distanceSensor2.getDistance(DistanceUnit.INCH);
            if (strafeChecking){
                if (distance + 1.5 > initialDistance){
                    telemetry.addLine("Pixel found? " + drive.getPoseEstimate().getX());
                }
                else if (distance - 1.5 > initialDistance){
                    telemetry.addLine("Pixel passed by? ");
                }
                else{
                    telemetry.addLine("starin at wall");
                }
            }
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
    public void outtake(){
        ElapsedTime timer1 = new ElapsedTime();
        robot.Arm.setPosition(armState.outtaking);
        robot.wrist.setPosition(armState.outtaking);
        robot.Claw.afterTape();
    }
}