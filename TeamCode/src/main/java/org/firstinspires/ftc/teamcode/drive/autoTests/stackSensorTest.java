package org.firstinspires.ftc.teamcode.drive.autoTests;

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

@Autonomous(name = "stack sensor test", group = "Auto")
public class stackSensorTest extends LinearOpMode {

    DistanceSensor distanceSensor, distanceSensor2;
    Robot robot;
    double distance, distance2;
    boolean strafeChecking = false;
    double initialDistance;
    ArrayList<Double> distances = new ArrayList<>();
    Pose2d currentPose = new Pose2d(50,0,Math.toRadians(-90));
    Pose2d stackFound = null;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(currentPose);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        distance = distanceSensor.getDistance(DistanceUnit.INCH);
        distance2 = distanceSensor2.getDistance(DistanceUnit.INCH);
        double y = -(distance2) + 11;
        telemetry.addLine(distance2 + "");
        TrajectorySequence lignUp = drive.trajectorySequenceBuilder(new Pose2d(50,0,Math.toRadians(-90)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))

                .lineToConstantHeading(new Vector2d(50,y))
                .addDisplacementMarker(()-> {
                    strafeChecking = true;
                })
                .lineToConstantHeading(new Vector2d(35,y))
                .build();
        drive.followTrajectorySequenceAsync(lignUp);
        waitForStart();
        if(isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()){
            distance = distanceSensor.getDistance(DistanceUnit.INCH);
            distance2 = distanceSensor.getDistance(DistanceUnit.INCH);
            if (strafeChecking){
                if (initialDistance < 1)initialDistance = distance - 1;
                distances.add(distance);
                if (distance + .25 < initialDistance){
                    telemetry.addLine("Pixel found? " + distance + "  " + initialDistance);
                    stackFound = drive.getPoseEstimate();
                    strafeChecking = false;
                }
                else{
                    telemetry.addLine("starin at wall  "+ distance+ "  " + initialDistance);
                }
            }
            if (stackFound != null){
                double stackDetectX = stackFound.getX() + 5;
                double stackDetectY = stackFound.getY() - distance2 -1.25;
                telemetry.addLine(stackFound.getY() + "  " + distance2);
                TrajectorySequence t = drive.trajectorySequenceBuilder(stackFound)
                                .lineToConstantHeading(new Vector2d(stackDetectX,y))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))

                                .lineToConstantHeading(new Vector2d(stackDetectX,stackDetectY)).build();
                drive.followTrajectorySequenceAsync(t);
                stackFound = null;
            }
            currentPose = drive.getPoseEstimate();
            telemetry.update();
            drive.update();
        }
    }
}