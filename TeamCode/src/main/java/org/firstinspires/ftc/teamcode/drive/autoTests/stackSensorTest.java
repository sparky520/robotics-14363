package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.SubSystems.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Autonomous(name = "short Blue Park", group = "Auto")
public class stackSensorTest extends LinearOpMode {

    DistanceSensor distanceSensor;
    Robot robot;
    double distance;
    boolean strafeChecking = false;
    double initialDistance;
    ArrayList<Double> distances = new ArrayList<>();
    Pose2d currentPose = new Pose2d(50,0,Math.toRadians(-90));
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(currentPose);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distance = distanceSensor.getDistance(DistanceUnit.INCH);
        Trajectory lignUp = drive.trajectoryBuilder(new Pose2d(50,-5,Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(50,-5 - (distance-11)))
                .addDisplacementMarker(()-> {
                    strafeChecking = true;
                })
                .lineToConstantHeading(new Vector2d(42,-5 - (distance-11)))
                .build();
        drive.followTrajectoryAsync(lignUp);
        waitForStart();
        if(isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()){
            if (strafeChecking){
                distances.add(distance);
                if (distance + 1.5 > initialDistance){
                    telemetry.addLine("Pixel found? " + distance + "  " + initialDistance);
                }
                else if (distance - 1.5 > initialDistance){
                    telemetry.addLine("Pixel passed by? "+ distance+ "  " + initialDistance);
                }
                else{
                    telemetry.addLine("starin at wall  "+ distance+ "  " + initialDistance);
                }
            }
            currentPose = drive.getPoseEstimate();
            distance = distanceSensor.getDistance(DistanceUnit.INCH);
            telemetry.update();
            drive.update();
        }
    }
}