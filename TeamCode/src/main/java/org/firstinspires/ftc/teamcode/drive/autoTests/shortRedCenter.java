package org.firstinspires.ftc.teamcode.drive.autoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class shortRedCenter extends LinearOpMode {
    enum state{
        toTape, wait1, toBoard, wait3, moveAwayFromBoard, retractArm, toStack, toStack2, wait4, IDLE,
    }
    Robot robot;
    state currentState = state.IDLE;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    DistanceSensor distanceSensor;
    Pose2d start = new Pose2d(0, 0, Math.toRadians(0));
    SampleMecanumDrive drive;
    double distanceSums = 0;
    int totalDistanceRecordings = 0;
    double boardOffset = 3;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);
        robot = new Robot(hardwareMap, telemetry);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distancesensor");

        Trajectory tape = drive.trajectoryBuilder(start).lineToLinearHeading(new Pose2d(38,-6,Math.toRadians(90))).build();
        Trajectory board = drive.trajectoryBuilder(tape.end()).lineToConstantHeading(new Vector2d(22,-25)).build();
        Trajectory moveAwayFromBoard = drive.trajectoryBuilder(board.end()).lineToConstantHeading(new Vector2d(22,-19)).build();
        Trajectory toStack = drive.trajectoryBuilder(moveAwayFromBoard.end()).lineToConstantHeading(new Vector2d(22,40)).build();
        Trajectory toStack2 = drive.trajectoryBuilder(toStack.end()).lineToConstantHeading(new Vector2d(22,45)).build();
        waitForStart();

        if (isStopRequested()) return;
        currentState = state.toTape;
        robot.Claw.setPosition(armState.intakingCLAW);
        drive.followTrajectoryAsync(tape);

        while (opModeIsActive() && !isStopRequested()) {
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
                        timer2.reset();
                    }
                    break;
                case toBoard:
                    if (!drive.isBusy()){
                        ElapsedTime timer2 = new ElapsedTime();
                        timer2.reset();
                        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
                        while (timer2.seconds()<.6){
                            if (distance < 20){
                                totalDistanceRecordings +=1;
                                distanceSums += distanceSensor.getDistance(DistanceUnit.INCH);
                            }
                        }
                        drive.followTrajectoryAsync(createPathToBoard(board.end()));
                        currentState = state.wait3;
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

            drive.update();
        }
    }
    public void outtakeAfterMedium(){
        ElapsedTime timer1 = new ElapsedTime();
        while (timer1.seconds() < .6){
            continue;
        }
        robot.Arm.setPosition(armState.high);
        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
    }
    public Trajectory createPathToBoard(Pose2d endPose){
        double averageDistance = distanceSums/totalDistanceRecordings;
        double finalDistance = averageDistance - boardOffset;
        Trajectory createdPath = drive.trajectoryBuilder(endPose).forward(-finalDistance).build();
        telemetry.addData("final", finalDistance);
        telemetry.update();
        return createdPath;
    }
    public void detectDistances(){
        double x = distanceSensor.getDistance(DistanceUnit.INCH);
        if (x < 20){
            totalDistanceRecordings +=1;
            distanceSums += x;
        }
    }
}