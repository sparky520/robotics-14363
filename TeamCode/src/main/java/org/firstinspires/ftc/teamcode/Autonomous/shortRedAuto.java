package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.SubSystems.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.arm;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class shortRedAuto extends LinearOpMode
{
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
                .lineToConstantHeading(new Vector2d(27,0))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                })
                .waitSeconds(2)
                .addDisplacementMarker(() -> {
                    //robot.Arm.setPosition(armState.outtaking);
                })
                .lineToLinearHeading(new Pose2d(19,-34,Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.outtaking);
                })
                .addDisplacementMarker(() -> {
                    //robot.Arm.setPosition(armState.medium);
                })
                .waitSeconds(2)
                .lineToConstantHeading(new Vector2d(-5,-38))
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToLinearHeading(new Pose2d(28,-4,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(28,2,Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    //robot.Arm.setPosition(armState.outtaking);
                })
                .lineToConstantHeading(new Vector2d(23,-42))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(0,-42))
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(newStart)
                .lineToLinearHeading(new Pose2d(31,0,Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(2,-10,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(23,-39))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(0,-42))
                .build();
        waitForStart();
        TrajectorySequence test = drive.trajectorySequenceBuilder(newStart)
                .turn(Math.toRadians(90))
                .build();
        if(isStopRequested()) return;
        drive.followTrajectorySequence(center);



    }
}