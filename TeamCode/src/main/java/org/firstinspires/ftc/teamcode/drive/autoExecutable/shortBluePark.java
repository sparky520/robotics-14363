package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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

import java.util.ArrayList;

@Autonomous(name = "short Blue Park", group = "Auto")
public class shortBluePark extends LinearOpMode {

    boolean tagFound = false;
    boolean caseTagFound = false;
    Robot robot;
    ElapsedTime intake = new ElapsedTime();
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
    boolean armRaised = false;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        Trajectory tape = drive.trajectoryBuilder(start).lineToConstantHeading(new Vector2d(0,1.1)).build();
        TrajectorySequence board1 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,1)).build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(board1.end()).lineToConstantHeading(new Vector2d(0,1.2)).build();
        TrajectorySequence stack1 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,1.5)).build();
        TrajectorySequence checkStack1 = drive.trajectorySequenceBuilder(new Pose2d(0,1.6)).lineToConstantHeading(new Vector2d(0,2)).build();
        TrajectorySequence board2 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,2.5)).build();
        TrajectorySequence checkBoard2 = drive.trajectorySequenceBuilder(checkStack1.end()).lineToConstantHeading(new Vector2d(0,3)).build();

        initColorDetection();
        waitForStart();
        camera.stopStreaming();
        initAprilTagDetect();

        if (isStopRequested()) return;
        telemetry.addLine(blueDetection.getLocation() + "");
        telemetry.update();
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
            boardXOffset = 6;
            boardYOffset = -2;
            boardPose = new Pose2d(42,43,Math.toRadians(-90));
        }

        currentState = state.board1;
        robot.Arm.setPosition(armState.low);
        robot.wrist.setPosition(armState.intakingCLAW);
        drive.followTrajectoryAsync(tape);
        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case board1:
                    if (tagOfInterest != null && drive.isBusy()){
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX()+ boardXOffset - (100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY() + boardYOffset +(100*tagOfInterest.pose.z/6);
                        caseTagFound = true;
                    }
                    if (!drive.isBusy()){
                        robot.Arm.setPosition(armState.outtaking);
                        robot.Claw.afterTape();
                        board1 = drive.trajectorySequenceBuilder(tape.end())
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
                    if (!drive.isBusy()){
                        robot.Claw.dropBoard();
                        pixelDropped = true;
                    }
                    if (!drive.isBusy() && pixelDropped){
                        pixelDropped = false;
                        stack1 = drive.trajectorySequenceBuilder(boardPose)
                                .addTemporalMarker(.5,() -> {
                                    robot.wrist.stack();
                                    robot.Arm.topStack();
                                    robot.Claw.setPosition(armState.close);
                                })
                                .addTemporalMarker(1.5,() -> {
                                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TOPSTACK);
                                })
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .waitSeconds(.2)
                                .splineToConstantHeading(new Vector2d(55,22), Math.toRadians(280))
                                .splineToConstantHeading(new Vector2d(55,5),Math.toRadians(280))
                                .build();
                        drive.setPoseEstimate(boardPose);
                        drive.followTrajectorySequenceAsync(stack1);
                        location = 9;
                        currentState = state.checkStack1;
                        timer.reset();
                    }
                    break;
                case checkStack1:
                    if (!drive.isBusy()){
                        robot.Claw.setPosition(armState.open);
                        checkStack1 = drive.trajectorySequenceBuilder(new Pose2d(55,5,Math.toRadians(-90)))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(58,-16))
                                .addTemporalMarker(2,() ->{
                                    robot.Claw.setPosition(armState.close);
                                })
                                .build();
                        drive.followTrajectorySequenceAsync(checkStack1);
                        caseTagFound = false;
                        currentState = state.board2;
                        tagOfInterest = null;
                    }
                    break;
                case board2:
                    if (!drive.isBusy()){
                        board2 = drive.trajectorySequenceBuilder(new Pose2d(58,-16,Math.toRadians(-90)))
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
                    telemetry.addLine("FOUND!!");
                    tagFound = true;
                    break;
                }
            }
        }
        else{
            telemetry.addLine("NOT FOUND");
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