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
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class colorSensorTest extends LinearOpMode
{
    Robot robot;
    SampleMecanumDrive drive;
    Trajectory p;
    @Override
    public void runOpMode() {
        ColorRangeSensor sensor = hardwareMap.get(ColorRangeSensor.class, "colorsensor");
        robot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d start = new Pose2d();

        if(isStopRequested()) return;
        while (true){
            double lum = 0.2126*sensor.red() + 0.7152*sensor.green() + 0.0722*sensor.blue();
            telemetry.addLine("Red: " + sensor.red());
            telemetry.addLine("Blue: " + sensor.blue());
            telemetry.addLine("Green: " + sensor.green());
            telemetry.update();
        }


    }
    public Trajectory test(Pose2d end){
        return drive.trajectoryBuilder(end).forward(10).build();
    }
}
