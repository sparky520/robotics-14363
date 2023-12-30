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
public class longBluePark extends LinearOpMode
{
    Robot robot;
    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d newStart = new Pose2d();
        TrajectorySequence center = drive.trajectorySequenceBuilder(newStart)

                .lineTo(new Vector2d(22,0))

                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(23,38))

                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(0,38))
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(newStart)
                .lineTo(new Vector2d(27,0))
                .turn(Math.toRadians(270))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(17,38))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(3,38))
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(newStart)
                .lineTo(new Vector2d(17,0))
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(29,38))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(0,38))
                .build();
        waitForStart();

        if(isStopRequested()) return;
        drive.followTrajectorySequence(center);



    }
}
