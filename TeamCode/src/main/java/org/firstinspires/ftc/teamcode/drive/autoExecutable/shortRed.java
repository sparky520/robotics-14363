package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.autoTests.shortRedCenter;
import org.firstinspires.ftc.teamcode.drive.autoTests.shortRedLeft;
import org.firstinspires.ftc.teamcode.drive.autoTests.shortRedRight;
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

@Autonomous(name = "short Red", group = "Auto")
public class shortRed extends LinearOpMode {
    enum state{
        toTape,
        wait1,
        wait2,
        toBoard,
        wait3,
        moveAwayFromBoard,
        retractArm,
        toStack,
        toStack2,
        wait4,
        IDLE,
    }
    Robot robot;
    state currentState = state.IDLE;
    ElapsedTime timer = new ElapsedTime();
    DistanceSensor distanceSensor;
    Pose2d start = new Pose2d(0, 0, Math.toRadians(0));
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    shortRedObjectDetect blueDetection;
    String webcamName;
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);
        robot = new Robot(hardwareMap, telemetry);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distancesensor");

        initCam();
        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if (blueDetection.getLocation().equals("LEFT")){
                TrajectorySequence tape = drive.trajectorySequenceBuilder(start)
                        .lineToLinearHeading(new Pose2d(25,0,Math.toRadians(90)))
                        .lineToConstantHeading(new Vector2d(28,6))
                        .build();
                Trajectory board = drive.trajectoryBuilder(tape.end()).lineToConstantHeading(new Vector2d(29.5,-30)).build();
                Trajectory moveAwayFromBoard = drive.trajectoryBuilder(board.end()).lineToConstantHeading(new Vector2d(35,-23)).build();
                //Trajectory toStack = drive.trajectoryBuilder(moveAwayFromBoard.end()).lineToConstantHeading(new Vector2d(19.5,15)).build();
                //Trajectory toStack2 = drive.trajectoryBuilder(moveAwayFromBoard.end()).lineToConstantHeading(new Vector2d(18.7,19)).build();

                currentState = state.toTape;
                robot.Claw.setPosition(armState.intakingCLAW);
                drive.followTrajectorySequenceAsync(tape);
                switch(currentState){
                    case toTape:
                        if (!drive.isBusy()){
                            currentState = state.wait1;
                            robot.Claw.setTape();
                            timer.reset();
                        }
                        break;
                    case wait1:
                        if (timer.seconds() > .3){
                            robot.Arm.setPosition(armState.medium);
                            outtakeAfterMedium();
                            drive.followTrajectoryAsync(board);
                            currentState = state.toBoard;
                        }
                        break;
                    case toBoard:
                        if (!drive.isBusy()){
                            drive.followTrajectory(toBoard(board));
                            telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
                            telemetry.update();
                        }
                    case wait3:
                        if (!drive.isBusy()){
                            robot.Claw.dropBoard();
                            currentState = state.moveAwayFromBoard;
                            timer.reset();
                        }
                        break;
                    case moveAwayFromBoard:
                        if (timer.seconds() > .3) {
                            drive.followTrajectoryAsync(moveAwayFromBoard);
                            currentState = state.retractArm;
                            timer.reset();
                        }
                        break;
                    case retractArm:
                        if (timer.seconds() > .5){
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                            robot.Arm.setPosition(armState.medium);
                            robot.Claw.setPosition(armState.intakingCLAW);
                            currentState = state.toStack;
                        }
                        break;
                    case toStack:
                        if (!drive.isBusy()){
                            //drive.followTrajectoryAsync(toTape);
                            robot.Arm.topStack();
                            currentState = state.toStack2;
                        }
                    case toStack2:
                        if (!drive.isBusy()){
                            //drive.followTrajectoryAsync(toTape2);
                            currentState = state.wait4;
                        }
                    case wait4:
                        if (!drive.isBusy()){
                            robot.Claw.setPosition(armState.intakingCLAW);
                            currentState = state.IDLE;
                        }
                    case IDLE:
                        break;
                }
            }else if(blueDetection.getLocation().equals("MIDDLE")){
                Trajectory tape = drive.trajectoryBuilder(start).lineToLinearHeading(new Pose2d(31.5,-6,Math.toRadians(90))).build();
                Trajectory board = drive.trajectoryBuilder(tape.end()).lineToConstantHeading(new Vector2d(19.5,-25)).build();
                Trajectory moveAwayFromBoard = drive.trajectoryBuilder(board.end()).lineToConstantHeading(new Vector2d(19.5,-23)).build();
                Trajectory toStack = drive.trajectoryBuilder(moveAwayFromBoard.end()).lineToConstantHeading(new Vector2d(19.5,15)).build();
                Trajectory toStack2 = drive.trajectoryBuilder(moveAwayFromBoard.end()).lineToConstantHeading(new Vector2d(18.7,19)).build();

                currentState = state.toTape;
                robot.Claw.setPosition(armState.intakingCLAW);
                drive.followTrajectoryAsync(tape);
                switch(currentState){
                    case toTape:
                        if (!drive.isBusy()){
                            currentState = state.wait1;
                            robot.Claw.setTape();
                            timer.reset();
                        }
                        break;
                    case wait1:
                        if (timer.seconds() > .3){
                            robot.Arm.setPosition(armState.medium);
                            outtakeAfterMedium();
                            drive.followTrajectoryAsync(board);
                            currentState = state.toBoard;
                        }
                        break;
                    case toBoard:
                        if (!drive.isBusy()){
                            drive.followTrajectory(toBoard(board));
                            telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
                            telemetry.update();
                        }
                    case wait3:
                        if (!drive.isBusy()){
                            robot.Claw.dropBoard();
                            currentState = state.moveAwayFromBoard;
                            timer.reset();
                        }
                        break;
                    case moveAwayFromBoard:
                        if (timer.seconds() > .3) {
                            drive.followTrajectoryAsync(moveAwayFromBoard);
                            currentState = state.retractArm;
                            timer.reset();
                        }
                        break;
                    case retractArm:
                        if (timer.seconds() > .5){
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                            robot.Arm.setPosition(armState.medium);
                            robot.Claw.setPosition(armState.intakingCLAW);
                            currentState = state.toStack;
                        }
                        break;
                    case toStack:
                        if (!drive.isBusy()){
                            drive.followTrajectoryAsync(toStack);
                            robot.Arm.topStack();
                            currentState = state.toStack2;
                        }
                    case toStack2:
                        if (!drive.isBusy()){
                            drive.followTrajectoryAsync(toStack2);
                            currentState = state.wait4;
                        }
                    case wait4:
                        if (!drive.isBusy()){
                            robot.Claw.setPosition(armState.intakingCLAW);
                            currentState = state.IDLE;
                        }
                    case IDLE:
                        break;
                }
            }else if(blueDetection.getLocation().equals("RIGHT")){
                Trajectory tape = drive.trajectoryBuilder(start).lineToLinearHeading(new Pose2d(32,-15,Math.toRadians(90))).build();
                Trajectory board = drive.trajectoryBuilder(tape.end()).lineToConstantHeading(new Vector2d(15.5,-29)).build();
                Trajectory moveAwayFromBoard = drive.trajectoryBuilder(board.end()).lineToConstantHeading(new Vector2d(35,-23)).build();
                //Trajectory toStack = drive.trajectoryBuilder(moveAwayFromBoard.end()).lineToConstantHeading(new Vector2d(19.5,15)).build();
                //Trajectory toStack2 = drive.trajectoryBuilder(moveAwayFromBoard.end()).lineToConstantHeading(new Vector2d(18.7,19)).build();

                currentState = state.toTape;
                robot.Claw.setPosition(armState.intakingCLAW);
                drive.followTrajectoryAsync(tape);
                switch(currentState){
                    case toTape:
                        if (!drive.isBusy()){
                            currentState = state.wait1;
                            robot.Claw.setTape();
                            timer.reset();
                        }
                        break;
                    case wait1:
                        if (timer.seconds() > .3){
                            robot.Arm.setPosition(armState.medium);
                            outtakeAfterMedium();
                            robot.Claw.afterTape();
                            drive.followTrajectoryAsync(board);
                            currentState = state.toBoard;
                        }
                        break;
                    case toBoard:
                        if (!drive.isBusy()){
                            drive.followTrajectory(toBoard(board));
                            telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
                            telemetry.update();
                        }
                    case wait3:
                        if (!drive.isBusy()){
                            robot.Claw.dropBoard();
                            currentState = state.moveAwayFromBoard;
                            timer.reset();
                        }
                        break;
                    case moveAwayFromBoard:
                        if (timer.seconds() > .3) {
                            drive.followTrajectoryAsync(moveAwayFromBoard);
                            currentState = state.retractArm;
                            timer.reset();
                        }
                        break;
                    case retractArm:
                        if (timer.seconds() > .5){
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                            robot.Arm.setPosition(armState.medium);
                            robot.Claw.setPosition(armState.intakingCLAW);
                            currentState = state.toStack;
                        }
                        break;
                    case toStack:
                        if (!drive.isBusy()){
                            //drive.followTrajectoryAsync(toTape);
                            robot.Arm.topStack();
                            currentState = state.toStack2;
                        }
                    case toStack2:
                        if (!drive.isBusy()){
                            //drive.followTrajectoryAsync(toTape2);
                            currentState = state.wait4;
                        }
                    case wait4:
                        if (!drive.isBusy()){
                            robot.Claw.setPosition(armState.intakingCLAW);
                            currentState = state.IDLE;
                        }
                    case IDLE:
                        break;
                }
            }
            drive.update();
        }
    }
    private void initCam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = "Webcam 1";
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
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
    public void outtakeAfterMedium(){
        ElapsedTime timer1 = new ElapsedTime();
        while (timer1.seconds() < .6){
            continue;
        }
        robot.Arm.setPosition(armState.high);
        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
    }
    public Trajectory toBoard(Trajectory end){
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        double y = distance - 3.5;
        Trajectory x = drive.trajectoryBuilder(end.end()).forward(-y).build();
        return x;
    }
}