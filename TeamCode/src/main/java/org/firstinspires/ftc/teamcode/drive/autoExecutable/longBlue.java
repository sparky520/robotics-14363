package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.autoExecutable.objectDetections.longBlueObjectDetect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.SubSystems.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "long blue", group = "Auto")
public class longBlue extends LinearOpMode {
    Robot robot;
    DistanceSensor distanceSensor, distanceSensor2, distanceSensor3;
    ElapsedTime timer = new ElapsedTime();
    Pose2d start = new Pose2d(0, 0, Math.toRadians(180));
    OpenCvCamera camera;
    double aprilLoc;
    longBlueObjectDetect blueDetection;
    String parkType = "FAR";
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
    int pauseDuration = 0;
    SampleMecanumDrive drive;
    bluePaths pathCreator;
    String pathType = "2+2";
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap, false);
        drive.setPoseEstimate(start);
        pathCreator = new bluePaths();

        initHardware();
        initColorDetection();
        while (!opModeIsActive() && !isStopRequested()){
            if (gamepad1.dpad_left){
                parkType = "FAR";
            }
            if (gamepad1.dpad_right){
                parkType = "CENTER";
            }
            if (gamepad1.dpad_down){
                pauseDuration -= 1;
            }
            if (gamepad1.dpad_up){
                pauseDuration += 1;
            }
            if (gamepad1.left_bumper){
                pathType = "PARK";
            }
            if (gamepad1.right_bumper){
                pathType = "2+2";
            }
            telemetry.addData("Park Location", parkType);
            telemetry.addData("Pause time", pauseDuration);
            telemetry.addData("Path type", pathType);
            telemetry.addData("TSE location", blueDetection.getLocation());
            telemetry.update();
        }
        waitForStart();
        camera.stopStreaming();

        if (isStopRequested()) return;
        telemetry.addLine(blueDetection.getLocation() + "");
        telemetry.update();
        blueDetection.location = "LEFT";
        if (blueDetection.getLocation().equals("RIGHT"))
        {
            followingPath = "RIGHT";
            aprilLoc = 37;

        }else if(blueDetection.getLocation().equals("MIDDLE"))
        {
            followingPath = "MIDDLE";
            aprilLoc = 28;
        }
        else if (blueDetection.getLocation().equals("LEFT")){
            followingPath = "LEFT";
            aprilLoc = 17.5;
        }
        TrajectorySequence tape = pathCreator.tape(robot,drive,start,followingPath, "LONG");
        drive.followTrajectorySequenceAsync(tape);
        currentState = state.throughTruss;
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case throughTruss:
                    if (!drive.isBusy()){
                        Pose2d atWall = new Pose2d(distanceSide,distanceFront,drive.getPoseEstimate().getHeading());
                        drive.setPoseEstimate(atWall);
                        TrajectorySequence throughTruss = pathCreator.throughTruss(robot,drive,atWall,distanceSide,curCycle);
                        drive.followTrajectorySequenceAsync(throughTruss);
                        currentState = state.toBoard;
                    }
                    break;
                case toBoard:
                    if (!drive.isBusy()){
                        Pose2d boardReallign = new Pose2d(distanceSide, 93 - distanceBack,drive.getPoseEstimate().getHeading());
                        drive.setPoseEstimate(boardReallign);
                        TrajectorySequence board = pathCreator.toBoard(robot,drive,boardReallign,aprilLoc);
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
                    colorBoardVal = colorBoard.red() + colorBoard.green() + colorBoard.blue();
                    if (colorBoardVal > boardColorThreshold){
                        TrajectorySequence nextToWall = pathCreator.goNextToWall(robot,drive,drive.getPoseEstimate(),followingPath,parkType,pathType);
                        if (pathType.equals("PARK")){
                            currentState = state.IDLE;
                        }
                        else{
                            currentState = state.backThroughTruss;
                        }
                        drive.followTrajectorySequenceAsync(nextToWall);
                    }
                    break;
                case backThroughTruss:
                    if (!drive.isBusy()){
                        Pose2d wallReallign = new Pose2d(distanceSide,drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading());
                        drive.setPoseEstimate(wallReallign);
                        TrajectorySequence throughTruss = pathCreator.backThroughTruss(robot,drive,wallReallign,distanceSide);
                        drive.followTrajectorySequenceAsync(throughTruss);
                        currentState = state.goToStack;
                    }
                    break;
                case goToStack:
                    if (!drive.isBusy()){
                        Pose2d wallReallign = new Pose2d(distanceSide,distanceFront,drive.getPoseEstimate().getHeading());
                        drive.setPoseEstimate(wallReallign);
                        TrajectorySequence toStack = pathCreator.goToStack(robot,drive,wallReallign);
                        drive.followTrajectorySequenceAsync(toStack);
                        currentState = state.intakeStackAndReset;
                    }
                    break;
                case intakeStackAndReset:
                    claw1Val = claw1.blue()+claw1.red()+claw1.green();
                    claw2Val = claw2.blue()+claw2.red()+claw2.green();
                    if (distanceFront < 2.5){
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.INTAKE_STACK);
                    }
                    if (claw2Val > 300 || claw1Val > 300){
                        Pose2d stackReallign = new Pose2d(29.5,0,drive.getPoseEstimate().getHeading());
                        drive.setPoseEstimate(stackReallign);
                        TrajectorySequence goToWall = pathCreator.intakeStackAndReset(robot,drive,stackReallign);
                        drive.followTrajectorySequenceAsync(goToWall);
                        currentState = state.throughTruss;
                        curCycle += 1;
                        aprilLoc = 22;

                    }
                    break;
                case park:
                    if (distanceBack < 8.5){
                        Pose2d boardReallign = new Pose2d(distanceSide,93-distanceBack,drive.getPoseEstimate().getHeading());
                        drive.setPoseEstimate(boardReallign);
                        TrajectorySequence park = pathCreator.park(robot,drive,boardReallign,"FAR");
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
            updateTelemetry();
            telemetry.update();
            drive.update();
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
        telemetry.addLine("X: " + drive.getPoseEstimate().getX() + " Y: " + drive.getPoseEstimate().getY());
    }
}