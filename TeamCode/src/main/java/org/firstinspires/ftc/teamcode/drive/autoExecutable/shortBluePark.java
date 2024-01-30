package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.SubSystems.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Autonomous(name = "short Blue Park", group = "Auto")
public class shortBluePark extends LinearOpMode {

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
    shortBlueObjectDetect blueDetection;
    int location = 1;
    Pose2d currentPose;
    Pose2d boardPose;
    Pose2d stackPose = new Pose2d(50,-75,Math.toRadians(-90));
    AprilTagDetection tagOfInterest = null;
    state currentState = state.IDLE;
    double boardXOffset;
    double boardYOffset = 0;
    boolean pixelDropped = false;
    boolean intaking = false;
    enum state{
        IDLE, board1, stack1, park, checkStack1, board2, checkBoard2,
    }
    AprilTagDetection lastTOI = null;
    boolean armRaised = false;
    double distance, distance2;
    boolean strafeChecking = false;
    double initialDistance, y;
    Pose2d stackFound;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        Trajectory tape = drive.trajectoryBuilder(start).lineToConstantHeading(new Vector2d(0,1.1)).build();
        TrajectorySequence board1 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,1)).build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(board1.end()).lineToConstantHeading(new Vector2d(0,1.2)).build();
        TrajectorySequence  stack1;
        TrajectorySequence checkStack1 = drive.trajectorySequenceBuilder(new Pose2d(0,1.6)).lineToConstantHeading(new Vector2d(0,2)).build();
        TrajectorySequence board2 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,2.5)).build();
        TrajectorySequence checkBoard2 = drive.trajectorySequenceBuilder(checkStack1.end()).lineToConstantHeading(new Vector2d(0,3)).build();

        initColorDetection();
        waitForStart();
        camera.stopStreaming();
        initAprilTagDetect();

        if (isStopRequested()) return;

        if (blueDetection.getLocation().equals("LEFT")){
            tape = drive.trajectoryBuilder(start)
                    .addTemporalMarker(1,() -> {
                        robot.Claw.setTape();
                    })
                    .lineToLinearHeading(new Pose2d(33,12, Math.toRadians(-90))).build();
            boardXOffset = -5;
            boardYOffset = 1;
            boardPose = new Pose2d(23,43,Math.toRadians(-90));

        }else if(blueDetection.getLocation().equals("MIDDLE")){
            tape = drive.trajectoryBuilder(start)
                    .addTemporalMarker(1,() -> {
                        robot.Claw.setTape();
                    })
                    .lineToLinearHeading(new Pose2d(35,5, Math.toRadians(-90))).build();
            boardXOffset = -1;
            boardYOffset = 1;
            boardPose = new Pose2d(26,43,Math.toRadians(-90));

        }else if(blueDetection.getLocation().equals("RIGHT")){
            tape = drive.trajectoryBuilder(start)
                    .addTemporalMarker(1,() -> {
                        robot.Claw.setTape();
                    })
                    .lineToLinearHeading(new Pose2d(30,-10, Math.toRadians(-90))).build();
            boardXOffset = 2.5;
            boardYOffset = 0;
            boardPose = new Pose2d(42,43,Math.toRadians(-90));
        }

        currentState = state.board1;
        robot.Arm.setPosition(armState.low);
        robot.Claw.setPosition(armState.close);
        robot.wrist.auto();
        drive.followTrajectoryAsync(tape);
        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case board1:
                    if (tagOfInterest != null && drive.isBusy()){
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX() + boardXOffset - (100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY() + boardYOffset +(100*tagOfInterest.pose.z/6);
                        caseTagFound = true;
                    }
                    if (!drive.isBusy()){
                        robot.Arm.setPosition(armState.outtaking);
                        board1 = drive.trajectorySequenceBuilder(tape.end())
                                .addTemporalMarker(.25,() ->{
                                    robot.Claw.afterTape();
                                })
                                .addTemporalMarker(.75,() -> {
                                    robot.wrist.setPosition(armState.outtaking);
                                })
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(boardX,boardY))
                                .build();
                        timer.reset();
                        armRaised = true;
                        location = 3;
                        drive.followTrajectorySequenceAsync(board1);
                    }
                    if (armRaised && timer.seconds() > 1){
                        caseTagFound = false;
                        currentState = state.stack1;
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
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .waitSeconds(.2)
                                .splineToConstantHeading(new Vector2d(50,22), Math.toRadians(280))
                                .addDisplacementMarker(() -> {
                                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TOPSTACK);
                                })
                                .splineToConstantHeading(new Vector2d(50,-25),Math.toRadians(280))
                                .build();
                        drive.followTrajectorySequenceAsync(stack1);
                        currentState = state.checkStack1;
                        timer.reset();
                    }
                    break;
                case checkStack1:
                    if (!drive.isBusy() && !strafeChecking){
                        y = -(distance2) + 11;
                        telemetry.addLine(distance2 + "");
                        TrajectorySequence lignUp = drive.trajectorySequenceBuilder(new Pose2d(50,-25,Math.toRadians(-90)))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .addDisplacementMarker(()-> {
                                    strafeChecking = true;
                                })
                                .lineToConstantHeading(new Vector2d(35,-25))
                                .build();
                        drive.followTrajectorySequenceAsync(lignUp);
                    }
                    if (strafeChecking){
                        if (initialDistance < 1)initialDistance = distance - 1;
                        if (distance + .25 < initialDistance){
                            telemetry.addLine("Pixel found? " + distance + "  " + initialDistance);
                            stackFound = drive.getPoseEstimate();
                            strafeChecking = false;
                        }
                        else{
                            telemetry.addLine("starin at wall  "+ distance+ "  " + initialDistance);
                        }
                    }
                    if (stackFound != null){
                        stackDetectX = stackFound.getX() + 5;
                        stackDetectY = stackFound.getY() - distance2 -1.25;
                        telemetry.addLine(stackFound.getY() + "  " + distance2);
                        TrajectorySequence t = drive.trajectorySequenceBuilder(currentPose)
                                .addDisplacementMarker(()->{
                                    robot.Claw.stack();
                                })
                                .lineToConstantHeading(new Vector2d(stackDetectX,y))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(stackDetectX,stackDetectY)).build();
                        drive.followTrajectorySequenceAsync(t);
                        stackFound = null;
                        currentState = state.board2;
                    }
                    break;
                case board2:
                    if (!drive.isBusy()){
                        robot.Claw.setPosition(armState.close);
                        board2 = drive.trajectorySequenceBuilder(new Pose2d(stackDetectX,stackDetectY,Math.toRadians(-90)))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(55,70)).build();
                        drive.followTrajectorySequenceAsync(board2);
                        location = 3;
                        currentState = state.checkBoard2;
                    }
                    break;
                case checkBoard2:
                    if (tagOfInterest != null){
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX() - 32 - (100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY()- 3 +(100*tagOfInterest.pose.z/6);
                        checkBoard2 = drive.trajectorySequenceBuilder(new Pose2d(55,70,Math.toRadians(-90)))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
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
                    tagOfInterest = tag;
                    //telemetry.addLine("FOUND!!");
                    tagFound = true;
                    break;
                }
            }
        }
        else{
            //telemetry.addLine("NOT FOUND");
        }
    }
    private void initColorDetection() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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
        robot.Arm.setPosition(armState.outtaking);
        robot.wrist.setPosition(armState.outtaking);
    }
}