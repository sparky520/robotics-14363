package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class longBlueCenter {
    Robot robot;
    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d newStart = new Pose2d();
        TrajectorySequence center = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToConstantHeading(new Vector2d(25,-3))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToLinearHeading(new Pose2d(26,-15,-80))
                .lineToConstantHeading(new Vector2d(47,-15))
                .lineToConstantHeading(new Vector2d(47,40))
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                    robot.Arm.setPosition(armState.high);
                })
                .lineToConstantHeading(new Vector2d(50,41))
                .turn(Math.toRadians(170))
                .lineToConstantHeading(new Vector2d(31,41))
                .lineToConstantHeading(new Vector2d(31,54))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(25,44))
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                })
                .lineToConstantHeading(new Vector2d(0,50))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .lineToConstantHeading(new Vector2d(0,62))
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToLinearHeading(new Pose2d(26,-8,Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToConstantHeading(new Vector2d(26,-4))
                .lineToConstantHeading(new Vector2d(48,-4))
                .lineToConstantHeading(new Vector2d(48,70))
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                    robot.Arm.setPosition(armState.high);
                })
                .lineToConstantHeading(new Vector2d(29.5,66))
                .lineToConstantHeading(new Vector2d(29.5,78.5))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(29.5,70))
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                })
                .lineToConstantHeading(new Vector2d(0,70))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .lineToConstantHeading(new Vector2d(0,76))
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToLinearHeading(new Pose2d(26,-6,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(28,0))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToConstantHeading(new Vector2d(26,-4))
                .lineToConstantHeading(new Vector2d(48,-4))
                .lineToConstantHeading(new Vector2d(48,45))
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                    robot.Arm.setPosition(armState.high);
                })
                .turn(Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(26.5,45))
                .lineToConstantHeading(new Vector2d(26.5,62.5))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(26.5,50))
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                })
                .lineToConstantHeading(new Vector2d(4,56))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .lineToConstantHeading(new Vector2d(4,66))
                .build();
        waitForStart();

        if(isStopRequested()) return;
        drive.followTrajectorySequence(left);
}
