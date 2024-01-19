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
public class shortBlueCenter extends LinearOpMode {
    int cycle = 0;
    enum state{
        toTape, wait1, toBoard, wait3, moveAwayFromBoard, retractArm, toStack, toBoardFromStack1, toBoardFromStack2, IDLE, toBoardFromStack_,toBoardFromStack2_,
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
    double boardOffset = 6.5;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);
        robot = new Robot(hardwareMap, telemetry);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distancesensor");

        Trajectory tape = drive.trajectoryBuilder(start).lineToLinearHeading(new Pose2d(37,7,Math.toRadians(-90))).build();
        Trajectory board = drive.trajectoryBuilder(tape.end()).lineToConstantHeading(new Vector2d(24,32)).build();
        Trajectory sensorToBoard = drive.trajectoryBuilder(tape.end()).lineToConstantHeading(new Vector2d(24,32)).build();
        Trajectory moveAwayFromBoard = drive.trajectoryBuilder(sensorToBoard.end()).lineToConstantHeading(new Vector2d(24,5)).build();
        Trajectory toStack = drive.trajectoryBuilder(moveAwayFromBoard.end()).lineToConstantHeading(new Vector2d(26.5,-48)).build();
        Trajectory toBoardFromStack_ = drive.trajectoryBuilder(toStack.end()).lineToConstantHeading(new Vector2d(24,10)).build();
        Trajectory toBoardFromStack = drive.trajectoryBuilder(toBoardFromStack_.end()).lineToConstantHeading(new Vector2d(24,50)).build();
        Trajectory moveAwayFromBoard1 = drive.trajectoryBuilder(toBoardFromStack.end()).lineToConstantHeading(new Vector2d(24,35)).build();
        Trajectory toStack2 = drive.trajectoryBuilder(toBoardFromStack.end()).lineToConstantHeading(new Vector2d(26.5,-20)).build();
        Trajectory toBoardFromStack2_ = drive.trajectoryBuilder(toStack2.end()).lineToConstantHeading(new Vector2d(24,40)).build();
        Trajectory toBoardFromStack2 = drive.trajectoryBuilder(toBoardFromStack2_.end()).lineToConstantHeading(new Vector2d(24,80)).build();
        Trajectory moveAwayFromBoard2 = drive.trajectoryBuilder(toBoardFromStack2.end()).lineToConstantHeading(new Vector2d(24,65)).build();
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
                        if (timer2.seconds() > .6){
                            if (cycle == 0){
                                sensorToBoard = createPathToBoard(board.end());
                            }else if (cycle == 1){
                                sensorToBoard = createPathToBoard(toBoardFromStack.end());
                            }else if (cycle == 2){
                                sensorToBoard = createPathToBoard(toBoardFromStack2.end());
                            }
                            drive.followTrajectoryAsync(sensorToBoard);
                            currentState = state.wait3;
                        }
                    }
                case wait3:
                    if (!drive.isBusy()){
                        telemetry.addData("pose est", drive.getPoseEstimate());
                        telemetry.update();
                        robot.Claw.dropBoard();
                        currentState = state.moveAwayFromBoard;
                        timer.reset();
                    }
                    break;
                case moveAwayFromBoard:
                    if (timer.seconds() > .3) {
                        if (cycle == 0){
                            drive.followTrajectoryAsync(moveAwayFromBoard);
                        }else if (cycle == 1){
                            drive.followTrajectoryAsync(moveAwayFromBoard1);
                        }else if (cycle == 2){
                            drive.followTrajectoryAsync(moveAwayFromBoard2);
                        }
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
                        cycle+=1;
                        if (cycle == 1){
                            drive.followTrajectoryAsync(toStack);
                            robot.Arm.topStack();
                            robot.Claw.setPosition(armState.outtaking);
                            currentState = state.toBoardFromStack1;
                        }else if (cycle == 2){
                            drive.followTrajectoryAsync(toStack2);
                            robot.Arm.topStack();
                            robot.Claw.setPosition(armState.outtaking);
                            currentState = state.toBoardFromStack2;
                        }else{
                            currentState = state.IDLE;
                        }
                    }
                case toBoardFromStack1:
                    if (!drive.isBusy()){
                        robot.Arm.setPosition(armState.medium);
                        outtakeAfterMedium();
                        drive.followTrajectoryAsync(toBoardFromStack_);
                        currentState = state.toBoardFromStack_;
                    }
                case toBoardFromStack_:
                    if (!drive.isBusy()){
                        drive.followTrajectoryAsync(toBoardFromStack);
                        currentState = state.toBoard;
                    }
                case toBoardFromStack2:
                    if (!drive.isBusy()){;
                        robot.Arm.setPosition(armState.medium);
                        outtakeAfterMedium();
                        drive.followTrajectoryAsync(toBoardFromStack2_);
                        currentState = state.toBoardFromStack2_;
                    }
                case toBoardFromStack2_:
                    if (!drive.isBusy()){
                        drive.followTrajectoryAsync(toBoardFromStack2);
                        currentState = state.toBoard;
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
        Trajectory createdPath;
        double averageDistance = distanceSums/totalDistanceRecordings;
        double finalDistance = averageDistance - boardOffset;
        createdPath = drive.trajectoryBuilder(endPose).forward(-finalDistance).build();
        telemetry.addData("final", finalDistance);
        telemetry.update();
        return createdPath;
    }
}