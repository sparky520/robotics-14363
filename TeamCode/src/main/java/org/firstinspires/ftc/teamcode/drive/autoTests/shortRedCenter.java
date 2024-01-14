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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class shortRedCenter extends LinearOpMode
{
    Robot robot;
    enum state{
        toCenter,
        wait1,
        wait2,
        toBoard,
        wait3,
        moveAwayFromBoard,
        retractArm1,
        retractArm2,
        IDLE,
    }
    state currentState = state.IDLE;
    @Override
    public void runOpMode() {
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distancesensor");
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        Pose2d start = new Pose2d();
        TrajectorySequence center1 = drive.trajectorySequenceBuilder(start)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToLinearHeading(new Pose2d(38.5,-3,Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToConstantHeading(new Vector2d(38.5, -3.01))
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                    robot.Arm.setPosition(armState.high);
                })
                .lineToConstantHeading(new Vector2d(21,-36))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(21,-35.51))
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                    robot.Arm.topStack();
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToConstantHeading(new Vector2d(23,47))
                .addDisplacementMarker(() -> {
                    robot.Arm.topStack();
                    robot.Claw.stack();
                })
                .build();

        Trajectory center = drive.trajectoryBuilder(start).lineToLinearHeading(new Pose2d(38.5,-3,Math.toRadians(90))).build();
        Trajectory board = drive.trajectoryBuilder(center.end()).lineToConstantHeading(new Vector2d(21,-36)).build();
        Trajectory moveAwayFromBoard = drive.trajectoryBuilder(board.end()).lineToConstantHeading(new Vector2d(21,-29)).build();
        waitForStart();

        if(isStopRequested()) return;

        currentState = state.toCenter;
        drive.followTrajectoryAsync(center);

        while (opModeIsActive() && !isStopRequested()) {
            switch(currentState){
                case toCenter:
                    if (!drive.isBusy()){
                        currentState = state.wait1;
                        robot.Claw.setTape();
                        timer.reset();
                    }
                    break;
                case wait1:
                    if (timer.seconds() > .3){
                        robot.Arm.setPosition(armState.medium);
                        currentState = state.wait2;
                    }
                    break;
                case wait2:
                    if (timer.seconds() > .6){
                        currentState = state.toBoard;
                        robot.Arm.setPosition(armState.high);
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                        timer.reset();
                    }
                case toBoard:
                    if (timer.seconds() > .5){
                        currentState = state.wait3;
                        drive.followTrajectoryAsync(board);
                    }
                    break;
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
                        currentState = state.retractArm1;
                    }
                    break;
                case retractArm1:
                    if (!drive.isBusy()){
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                        robot.Arm.setPosition(armState.medium);
                        robot.Claw.setPosition(armState.intakingCLAW);
                        currentState = state.retractArm2;
                        timer.reset();
                    }
                    break;
                case retractArm2:
                    if (timer.seconds() > .5){
                        robot.Arm.topStack();
                        currentState = state.IDLE;
                    }
                case IDLE:
                    break;
            }
        }

        drive.update();
        telemetry.addData("state", currentState);
    }
}
