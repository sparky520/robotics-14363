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
    double dOffset;
    boolean initOffset = false;
    boolean firstStackFound = false;
    double totalDistance = 0;
    int totalChecks = 0;
    Pose2d strafePos;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, false );
        drive.setPoseEstimate(currentPose);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        distance = distanceSensor.getDistance(DistanceUnit.INCH);
        distance2 = distanceSensor2.getDistance(DistanceUnit.INCH);
        double y = -(distance2) + 9;
        telemetry.addLine(distance2 + "");
        TrajectorySequence lignUp = drive.trajectorySequenceBuilder(new Pose2d(50,0,Math.toRadians(-90)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(50,y))
                .addDisplacementMarker(()-> {
                    strafeChecking = true;
                    strafePos = drive.getPoseEstimate();
                })
                .lineToConstantHeading(new Vector2d(30,y))
                .build();
        drive.followTrajectorySequenceAsync(lignUp);
        waitForStart();
        if(isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()){
            distance = distanceSensor.getDistance(DistanceUnit.INCH);
            distance2 = distanceSensor2.getDistance(DistanceUnit.INCH);
            if (!(distance +distance2 < 40)){
                telemetry.addLine("Sensors not reading");
                break;
            }
            else{
                if (strafeChecking){
                    if (distance2 - distance  > .4){
                        telemetry.addLine("Pixel found? " + distance + "  " + initialDistance);
                        telemetry.addData("difference", distance2 - distance);
                        telemetry.addData("distance",distance);
                        telemetry.addData("distance2",distance2);
                        stackFound = currentPose;
                        totalChecks += 1;
                        totalDistance += stackFound.getX() - strafePos.getX();
                        firstStackFound = true;
                    }
                    else{
                        telemetry.addLine("starin at wall  "+ distance+ "  " + initialDistance);
                        telemetry.addLine(distance2 - distance  + "");
                        telemetry.addLine(distance+ "");
                        telemetry.addLine(distance2+ "");
                        if (firstStackFound){
                            if (totalChecks > 1){
                                strafeChecking = false;
                                double stackDetectX = stackFound.getX() + 6.25;
                                double stackDetectY = stackFound.getY() - distance2;
                                double averageDistance = strafePos.getX() + (totalDistance/totalChecks) + 8.5;
                                telemetry.addData("checks",totalChecks);
                                telemetry.addData("avgD",averageDistance);
                                telemetry.addLine(stackDetectX + "   " + stackDetectY);
                                TrajectorySequence t = drive.trajectorySequenceBuilder(stackFound)
                                        .lineToConstantHeading(new Vector2d(stackDetectX,y))
                                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))

                                        .lineToConstantHeading(new Vector2d(averageDistance,stackDetectY)).build();
                                drive.followTrajectorySequenceAsync(t);
                                stackFound = null;
                            }else{
                                /*
                                TrajectorySequence reStrafe = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))

                                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() + 15, y)).build();
                                firstStackFound = false;
                                drive.followTrajectorySequenceAsync(reStrafe);*/
                                telemetry.addData("Checks", totalChecks);
                            }

                        }
                    }
                }
            }
            currentPose = drive.getPoseEstimate();
            telemetry.update();
            drive.update();
        }
    }
}