package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
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

@Autonomous(name = "long Red 2 + 2", group = "Auto")
public class longRed2_2 extends LinearOpMode {
    Robot robot;
    DistanceSensor distanceSensor, distanceSensor2, distanceSensor3;
    ElapsedTime timer = new ElapsedTime();
    Pose2d start = new Pose2d(0, 0, Math.toRadians(180));
    OpenCvCamera camera;
    double aprilLoc;
    longRedObjectDetect blueDetection;
    state currentState = state.IDLE;
    enum state{
        IDLE, throughTruss,toBoard, goNextToWall, backThroughTruss,goToStack, intakeStackAndReset, park
    }
    double distanceFront, distanceSide, distanceBack;
    ColorRangeSensor claw1,claw2,colorBoard;
    String followingPath;
    double claw1Val,claw2Val,colorBoardVal;
    int curCycle = 0;
    int boardColorThreshold = 300;
    SampleMecanumDrive drive;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap, true);
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
                    .lineToLinearHeading(new Pose2d(28,-2,Math.toRadians(0)))
                    .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(-90)))
                    .build();
            aprilLoc = 20;
            drive.followTrajectorySequenceAsync(tape);

        }//else if(blueDetection.getLocation().equals("LEFT")){
        else if (false){
            followingPath = "LEFT";
            TrajectorySequence tape = drive.trajectorySequenceBuilder(start)
                    .addTemporalMarker(1,() -> {
                        robot.Claw.setTape();
                    })
                    .lineToSplineHeading(new Pose2d(28,6,Math.toRadians(90))).build();
            aprilLoc = 20;
            drive.followTrajectorySequenceAsync(tape);
        }

        robot.Claw.setPosition(armState.close);
        robot.wrist.setPosition(armState.intakingCLAW);
        robot.Arm.intake();
        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.STATION);
        currentState = state.throughTruss;
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case throughTruss:
                    if (!drive.isBusy()){
                        Pose2d atWall = new Pose2d(distanceSide,distanceFront,drive.getPoseEstimate().getHeading());
                        drive.setPoseEstimate(atWall);
                        TrajectorySequence throughTruss = drive.trajectorySequenceBuilder(atWall)
                                .addTemporalMarker(.1,() -> {
                                    robot.Arm.half();
                                })
                                .addTemporalMarker(2,()->{
                                    if (curCycle == 0){
                                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.AUTO1);
                                    }else{
                                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.MEDIUMIN);
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
                                .lineToSplineHeading(new Pose2d(8,75,Math.toRadians(-90)))
                                .splineToConstantHeading(new Vector2d(29,105),Math.toRadians(280)).build();
                        drive.followTrajectorySequenceAsync(throughTruss);
                        currentState = state.toBoard;
                    }
                    break;
                case toBoard:
                    if (!drive.isBusy()){
                        Pose2d boardReallign = new Pose2d(distanceSide, 93 - distanceBack,drive.getPoseEstimate().getHeading());
                        drive.setPoseEstimate(boardReallign);

                        TrajectorySequence board = drive.trajectorySequenceBuilder(boardReallign)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToLinearHeading(new Pose2d(aprilLoc,90,Math.toRadians(-90))).build();
                        drive.followTrajectorySequenceAsync(board);
                        if (curCycle == 1){
                            currentState = state.park;
                        }
                        else{
                            currentState = state.goNextToWall;
                        }
                    }
                    break;
                case goNextToWall:
                    if (colorBoardVal > boardColorThreshold){
                        robot.Claw.setPosition(armState.open);
                        TrajectorySequence nextToWall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(.5,() -> {
                                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TOPSTACK);
                                })
                                .addTemporalMarker(1,() -> {
                                    robot.wrist.topStack();
                                    robot.Arm.topStack();
                                })
                                .lineToConstantHeading(new Vector2d(0,62)).build();
                        if (curCycle == 1){
                            currentState = state.park;
                        }else{
                            drive.followTrajectorySequenceAsync(nextToWall);
                            currentState = state.backThroughTruss;
                        }
                    }
                    break;
                case backThroughTruss:
                    if (!drive.isBusy()){
                        Pose2d wallReallign = new Pose2d(distanceSide,drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading());
                        drive.setPoseEstimate(wallReallign);
                        TrajectorySequence throughTruss = drive.trajectorySequenceBuilder(wallReallign)
                                .lineToSplineHeading(new Pose2d(7,15,Math.toRadians(-90)))
                                .splineToConstantHeading(new Vector2d(25,-13),Math.toRadians(280)).build();
                        drive.followTrajectorySequenceAsync(throughTruss);
                        currentState = state.goToStack;
                    }
                    break;
                case goToStack:
                    if (!drive.isBusy()){
                        Pose2d wallReallign = new Pose2d(distanceSide,distanceFront,drive.getPoseEstimate().getHeading());
                        drive.setPoseEstimate(wallReallign);
                        TrajectorySequence toStack = drive.trajectorySequenceBuilder(wallReallign)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(21, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(29.5,0))
                                .build();
                        drive.followTrajectorySequenceAsync(toStack);
                        currentState = state.intakeStackAndReset;
                    }
                    break;
                case intakeStackAndReset:
                    if (distanceFront < 2.5){
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.STATION);
                    }
                    if (claw2Val > 300 || claw1Val > 300){
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.STATION);
                        robot.Claw.setPosition(armState.close);

                        drive.setPoseEstimate(new Pose2d(distanceSide,distanceFront,drive.getPoseEstimate().getHeading()));
                        TrajectorySequence goToWall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(5,29,Math.toRadians(-90)))
                                .build();

                        drive.followTrajectorySequenceAsync(goToWall);
                        currentState = state.throughTruss;
                        curCycle += 1;
                        if (followingPath.equals("MIDDLE") || followingPath.equals("LEFT")){
                            aprilLoc = 29;
                        }else{
                            aprilLoc = 16;
                            robot.Claw.closeRight();
                        }
                    }
                    break;
                case park:
                    if (distanceBack < 8.5){
                        robot.Claw.setPosition(armState.open);
                        Pose2d boardReallign = new Pose2d(distanceSide,93-distanceBack,drive.getPoseEstimate().getHeading());
                        drive.setPoseEstimate(boardReallign);
                        TrajectorySequence park = drive.trajectorySequenceBuilder(boardReallign)
                                .addTemporalMarker(.5,()->{
                                    robot.Arm.setPosition(armState.medium);
                                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.RESET);
                                    robot.wrist.setPosition(armState.intakingCLAW);
                                })
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
    private void initColorDetection() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        blueDetection = new longRedObjectDetect(telemetry);

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
    public void initHardware(){
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor4");
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
        telemetry.addLine("X: " + drive.getPoseEstimate().getX() + " Y: " + drive.getPoseEstimate().getY());
    }
}