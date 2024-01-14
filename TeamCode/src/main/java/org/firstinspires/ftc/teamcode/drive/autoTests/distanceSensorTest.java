package org.firstinspires.ftc.teamcode.drive.autoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class distanceSensorTest extends LinearOpMode
{
    Robot robot;
    SampleMecanumDrive drive;
    Trajectory p;
    @Override
    public void runOpMode() {
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distancesensor");
        robot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d();

        if(isStopRequested()) return;

        while (distanceSensor.getDistance(DistanceUnit.INCH) > 5){
            telemetry.addLine("oop");
            p = test(start);
            drive.followTrajectory(p);
            start = p.end();
        }
        telemetry.addLine("There u go");



    }
    public Trajectory test(Pose2d end){
        return drive.trajectoryBuilder(end).forward(10).build();
    }
}
