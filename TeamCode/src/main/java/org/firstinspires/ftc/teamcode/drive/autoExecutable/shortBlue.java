package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.autoExecutable.*;
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
    shortBlueObjectDetect blueDetection;
    int location = 5;
    Pose2d currentPose;
    Pose2d boardPose = new Pose2d(26,43,Math.toRadians(-90));
    Pose2d stackPose = new Pose2d(50,-75,Math.toRadians(-90));
    AprilTagDetection tagOfInterest = null;
    state currentState = state.IDLE;
    double boardXOffset;

    enum state{
        IDLE, board1, stack1, checkStack1, board2, checkBoard2,
    }
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        Trajectory tape = drive.trajectoryBuilder(start).lineToConstantHeading(new Vector2d(0,1.1)).build();
        TrajectorySequence board1 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,1)).build();
        TrajectorySequence stack1 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,1.5)).build();
        TrajectorySequence checkStack1 = drive.trajectorySequenceBuilder(new Pose2d(0,1.6)).lineToConstantHeading(new Vector2d(0,2)).build();
        TrajectorySequence board2 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,2.5)).build();
        TrajectorySequence checkBoard2 = drive.trajectorySequenceBuilder(checkStack1.end()).lineToConstantHeading(new Vector2d(0,3)).build();

        initColorDetection();
        waitForStart();
        camera.stopStreaming();
        initAprilTagDetect();

        String testing = "RIGHT";
        if (isStopRequested()) return;
        if (testing.equals("LEFT")){
            tape = drive.trajectoryBuilder(start)
                    .addDisplacementMarker(() -> {
                        //robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                        //robot.Arm.setPosition(armState.low);
                        //robot.Claw.setPosition(armState.intakingCLAW);
                    })
                    .lineToLinearHeading(new Pose2d(33,12, Math.toRadians(-90)))
                    .addDisplacementMarker(() -> {
                        robot.Claw.setTape();
                    }).build();
            boardXOffset = 3;

        }else if(testing.equals("MIDDLE")){
            tape = drive.trajectoryBuilder(start)
                    .addDisplacementMarker(() -> {
                        //robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                        //robot.Arm.setPosition(armState.low);
                        //robot.Claw.setPosition(armState.intakingCLAW);
                    })
                    .lineToLinearHeading(new Pose2d(35,5, Math.toRadians(-90)))
                    .addDisplacementMarker(() -> {
                        robot.Claw.setTape();
                    }).build();
            boardXOffset = 0;

        }else if(testing.equals("RIGHT")){
            tape = drive.trajectoryBuilder(start)
                    .addDisplacementMarker(() -> {
                        //robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                        //robot.Arm.setPosition(armState.low);
                        //robot.Claw.setPosition(armState.intakingCLAW);
                    })
                    .lineToLinearHeading(new Pose2d(30,-10, Math.toRadians(-90)))
                    .addDisplacementMarker(() -> {
                        robot.Claw.setTape();
                    }).build();
            boardXOffset = -3;
        }

        currentState = state.board1;
        //robot.Claw.setPosition(armState.intakingCLAW);
        drive.followTrajectoryAsync(tape);
        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case board1:
                    if (tagOfInterest != null && caseTagFound == false){
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX()- boardXOffset - (100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY()- 8 +(100*tagOfInterest.pose.z/6);
                        board1 = drive.trajectorySequenceBuilder(tape.end())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(boardX,boardY))
                                .build();
                        caseTagFound = true;
                    }
                    if (!drive.isBusy() && caseTagFound == true){
                        location = 3;
                        drive.followTrajectorySequenceAsync(board1);
                        caseTagFound = false;
                        currentState = state.stack1;
                        tagOfInterest = null;
                    }
                    break;
                case stack1:
                    if (!drive.isBusy()){
                        stack1 = drive.trajectorySequenceBuilder(boardPose)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .waitSeconds(.2)
                                .splineToConstantHeading(new Vector2d(53,22), Math.toRadians(280))
                                .splineToConstantHeading(new Vector2d(53,-13),Math.toRadians(280))
                                .build();
                        drive.setPoseEstimate(boardPose);
                        drive.followTrajectorySequenceAsync(stack1);
                        camera.closeCameraDevice();
                        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
                        initAprilTagDetect();
                        location = 10;
                        currentState = state.checkStack1;

                        timer.reset();
                    }
                    break;
                case checkStack1:
                    if (tagOfInterest != null && caseTagFound == false && timer.seconds() > 3){
                        Pose2d toBoardEnd = currentPose;
                        boardX = toBoardEnd.getX() - (100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY() - 7 - (100*tagOfInterest.pose.z/6);
                        checkStack1 = drive.trajectorySequenceBuilder(new Pose2d(49,-25,Math.toRadians(-90)))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(boardX,boardY))
                                .build();
                        caseTagFound = true;
                    }
                    if (!drive.isBusy() && caseTagFound == true){
                        drive.followTrajectorySequenceAsync(checkStack1);
                        caseTagFound = false;
                        currentState = state.board2;
                        tagOfInterest = null;
                    }
                    break;
                case board2:
                    if (!drive.isBusy()){
                        drive.setPoseEstimate(new Pose2d(50,-75,Math.toRadians(-90)));

                        board2 = drive.trajectorySequenceBuilder(stackPose)
                                .lineToConstantHeading(new Vector2d(50,-20)).build();
                        drive.followTrajectorySequenceAsync(board2);
                        camera.closeCameraDevice();
                        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
                        initAprilTagDetect();
                        location = 3;
                        currentState = state.checkBoard2;
                    }
                    break;
                case checkBoard2:
                    if (tagOfInterest != null && caseTagFound == false){
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX()-10 - (100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY()- 8 +(100*tagOfInterest.pose.z/6);
                        checkBoard2 = drive.trajectorySequenceBuilder(tape.end())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(boardX,boardY))
                                .build();
                        caseTagFound = true;
                    }
                    if (!drive.isBusy() && caseTagFound == true){
                        location = 3;
                        drive.followTrajectorySequenceAsync(checkBoard2);
                        caseTagFound = false;
                        currentState = state.stack1;
                        tagOfInterest = null;
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
                    tagFound = true;
                    break;
                }
            }
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
}