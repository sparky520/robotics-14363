package org.firstinspires.ftc.teamcode.drive.autoTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.SubSystems.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "short Blue", group = "Auto")
public class aprilTagTest extends LinearOpMode {
    OpenCvCamera camera;
    aprilTagDetection blueDetection;
    String webcamName;
    Robot robot;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d newStart = new Pose2d();
        TrajectorySequence center = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToLinearHeading(new Pose2d(37,7,Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToConstantHeading(new Vector2d(36,15.01))
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                    robot.Arm.setPosition(armState.high);
                })
                .lineToConstantHeading(new Vector2d(31,15.02))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(24,34.5))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(26,24))
                //.waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                })
                .lineToConstantHeading(new Vector2d(0,29))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(0,36))
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToLinearHeading(new Pose2d(31,17.5,Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToConstantHeading(new Vector2d(31,17.51))
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                    robot.Arm.setPosition(armState.high);
                })
                .lineToConstantHeading(new Vector2d(31,17.52))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(22.5,36.75))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(20,24))
                //.waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                })
                .lineToConstantHeading(new Vector2d(0,29))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(0,36))
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToLinearHeading(new Pose2d(28,10,Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(28,-5))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToConstantHeading(new Vector2d(28,-4.99))
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                    robot.Arm.setPosition(armState.high);
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                //.lineToConstantHeading(new Vector2d(31,9.02))
                //.waitSeconds(1)
                .lineToConstantHeading(new Vector2d(33.75,36.75))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(33,24))
                //.waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                })
                .lineToConstantHeading(new Vector2d(0,23))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(0,33))
                .build();
        initCam();
        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;
        if (blueDetection.getLocation().equals("LEFT")){
            drive.followTrajectorySequence(left);
        }else if(blueDetection.getLocation().equals("MIDDLE")){
            drive.followTrajectorySequence(center);
        }else if(blueDetection.getLocation().equals("RIGHT")){
            drive.followTrajectorySequence(right);
        }
        else{
            telemetry.addLine("wrong" + blueDetection.getLocation());
        }
        telemetry.update();
    }
    private void initCam() {

        //This line retrieves the resource identifier for the camera monitor view. The camera monitor view is typically used to display the camera feed
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = "Webcam 1";

        // This line creates a webcam instance using the OpenCvCameraFactor with the webcam name (webcamName) and the camera monitor view ID.
        // The camera instance is stored in the camera variable that we can use later
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        // initializing our Detection class (details on how it works at the top)
        blueDetection = new aprilTagDetection(telemetry);

        // yeah what this does is it gets the thing which uses the thing so we can get the thing
        /*
        (fr tho idk what pipeline does, but from what I gathered,
         we basically passthrough our detection into the camera
         and we feed the streaming camera frames into our Detection algorithm)
         */
        camera.setPipeline(blueDetection);

        /*
        this starts the camera streaming, with 2 possible combinations
        it starts streaming at a chosen res, or if something goes wrong it throws an error
         */
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.showFpsMeterOnViewport(true);
                camera.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Unspecified Error Occurred; Camera Opening");
            }
        });
    }
}