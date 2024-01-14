package org.firstinspires.ftc.teamcode.drive.autoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    @Override
    public void runOpMode() {
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distancesensor");
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
        waitForStart();

        if(isStopRequested()) return;
        drive.followTrajectorySequence(center1);




    }
}
