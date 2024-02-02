package org.firstinspires.ftc.teamcode.drive.autoExecutable;

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

@Autonomous(name = "short red Park", group = "Auto")
public class shortRedPark extends LinearOpMode {

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
    shortRedObjectDetect blueDetection;
    int location = 5;
    Pose2d currentPose;
    Pose2d boardPose;
    Pose2d stackPose = new Pose2d(50,-75,Math.toRadians(-90));
    AprilTagDetection tagOfInterest = null;
    state currentState = state.IDLE;
    double boardXOffset;
    double boardYOffset = 0;
    boolean pixelDropped = false;

    enum state{
        IDLE, board1, stack1, park, checkStack1, board2, checkBoard2,
    }
    boolean armRaised = false;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        TrajectorySequence tape = drive.trajectorySequenceBuilder(start).lineToConstantHeading(new Vector2d(0,1.1)).build();
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
            tape = drive.trajectorySequenceBuilder(start)
                    .addTemporalMarker(2,() -> {
                        robot.Claw.setTape();
                    })
                    .lineToLinearHeading(new Pose2d(31,-12, Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(31,1))
                    .build();

            boardXOffset = 13;
            boardYOffset = 0;
            boardPose = new Pose2d(23,-43,Math.toRadians(90));

        }else if(blueDetection.getLocation().equals("MIDDLE")){
            tape = drive.trajectorySequenceBuilder(start)
                    .lineToLinearHeading(new Pose2d(37,-5, Math.toRadians(70))).build();
            boardXOffset = -14;
            boardYOffset = 4;
            boardPose = new Pose2d(26,-43,Math.toRadians(90));

        }else if(blueDetection.getLocation().equals("RIGHT")){
            tape = drive.trajectorySequenceBuilder(start)
                    .lineToLinearHeading(new Pose2d(30,-16, Math.toRadians(90))).build();
            boardXOffset = -4;
            boardYOffset = 0;
            boardPose = new Pose2d(29,-43,Math.toRadians(90));
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
                        boardX = toBoardEnd.getX()+ boardXOffset + 2 + (100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY() + boardYOffset - 2 - (100*tagOfInterest.pose.z/6);
                        board1 = drive.trajectorySequenceBuilder(tape.end())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToLinearHeading(new Pose2d(boardX,boardY,Math.toRadians(90)))
                                .build();
                        caseTagFound = true;
                    }
                    if (!drive.isBusy() && caseTagFound == true){
                        robot.Claw.setTape();
                        outtake();
                        timer.reset();
                        armRaised = true;
                        location = 5;
                        drive.followTrajectorySequenceAsync(board1);
                    }
                    if (armRaised && timer.seconds() > 1){
                        caseTagFound = false;
                        currentState = state.park;
                        tagOfInterest = null;
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.AUTO1);
                    }
                case park:
                    if (!drive.isBusy()){
                        robot.Claw.dropBoard();
                        park = drive.trajectorySequenceBuilder(currentPose)
                                .waitSeconds(1)
                                .forward(5)
                                .addDisplacementMarker(() -> {
                                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.STATION);
                                    robot.Arm.setPosition(armState.medium);
                                })
                                .turn(Math.toRadians(-100))
                                .back(20)
                                .addDisplacementMarker(() -> {
                                    robot.Claw.setPosition(armState.close);
                                })
                                .build();
                        drive.followTrajectorySequenceAsync(park);
                        currentState = state.IDLE;
                    }


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
        blueDetection = new shortRedObjectDetect(telemetry);

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