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
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class shortRedCenter extends LinearOpMode {
    enum state{
        toTape, wait1, wait2, wait2_5, toBoard, wait3, moveAwayFromBoard, retractArm, toStack, toStack2, wait4, IDLE,
    }
    Robot robot;
    state currentState = state.IDLE;
    ElapsedTime timer = new ElapsedTime();
    DistanceSensor distanceSensor;
    Pose2d start = new Pose2d(0, 0, Math.toRadians(0));
    SampleMecanumDrive drive;
    Trajectory toboard;
    double distanceSums = 0;
    int totalDistanceRecordings = 0;
    boolean activeDistanceSensor = false;
    double boardOffset = 2;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);
        robot = new Robot(hardwareMap, telemetry);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distancesensor");

        Trajectory tape = drive.trajectoryBuilder(start).lineToLinearHeading(new Pose2d(38,-6,Math.toRadians(90))).build();
        Trajectory board = drive.trajectoryBuilder(tape.end()).lineToConstantHeading(new Vector2d(19.5,-25)).build();
        Trajectory moveAwayFromBoard = drive.trajectoryBuilder(board.end()).lineToConstantHeading(new Vector2d(19.5,-23)).build();
        Trajectory toTape = drive.trajectoryBuilder(moveAwayFromBoard.end()).lineToConstantHeading(new Vector2d(19.5,15)).build();
        Trajectory toTape2 = drive.trajectoryBuilder(toTape.end()).lineToConstantHeading(new Vector2d(18.7,19)).build();
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
                        currentState = state.wait2;
                    }
                    break;
                case wait2:
                    telemetry.addLine("1");
                    if (!drive.isBusy()){
                        currentState = state.toBoard;
                        activeDistanceSensor = true;
                        timer.reset();
                    }
                case toBoard:
                    if (timer.seconds() > .5){
                        activeDistanceSensor = false;
                        telemetry.addLine("3");
                        drive.followTrajectoryAsync(createPathToBoard(board.end()));
                        currentState = state.wait3;
                    }
                case wait3:
                    if (!drive.isBusy()){
                        drive.setPoseEstimate(toboard.end());
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
                        drive.setPoseEstimate(moveAwayFromBoard.end());
                        drive.followTrajectoryAsync(toTape);
                        robot.Arm.topStack();
                        currentState = state.toStack2;
                    }
                case toStack2:
                    if (!drive.isBusy()){
                        drive.setPoseEstimate(toTape.end());
                        drive.followTrajectoryAsync(toTape2);
                        currentState = state.wait4;
                    }
                case wait4:
                    if (!drive.isBusy()){
                        drive.setPoseEstimate(toTape2.end());
                        robot.Claw.setPosition(armState.intakingCLAW);
                        currentState = state.IDLE;
                    }
                case IDLE:
                    break;
            }
            if (activeDistanceSensor){
                telemetry.addLine("3");
                double x = distanceSensor.getDistance(DistanceUnit.INCH);
                if (x < 20){
                    telemetry.addLine("4");
                    totalDistanceRecordings +=1;
                    distanceSums += x;
                }
            }
            telemetry.update();
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
        return createdPath;
    }
}