package org.firstinspires.ftc.teamcode.drive.autoExecutable;

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

@Autonomous(name = "long Blue", group = "Auto")
public class longBlue extends LinearOpMode {
    OpenCvCamera camera;
    longBlueObjectDetect blueDetection;
    String webcamName;
    Robot robot;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d newStart = new Pose2d();
        TrajectorySequence right = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .waitSeconds(.25)
                .lineToLinearHeading(new Pose2d(36,-13,Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToLinearHeading(new Pose2d(54,-4,Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(54,55))
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.AUTO_HIGH);
                    robot.Arm.longAuto();
                })
                .lineToConstantHeading(new Vector2d(48,55.01))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(33,72))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(33,65))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                })
                .lineToConstantHeading(new Vector2d(4,65))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(4,70))
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToLinearHeading(new Pose2d(26,-6,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(28,2))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToConstantHeading(new Vector2d(26,-4))
                .lineToLinearHeading(new Pose2d(48,-4,Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(51,55))
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.AUTO_HIGH);
                    robot.Arm.longAuto();
                })
                .lineToConstantHeading(new Vector2d(48,55.01))
                .waitSeconds(.75)
                .lineToConstantHeading(new Vector2d(22,74))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(30,60))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                })
                .lineToConstantHeading(new Vector2d(4,55))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(4,76))
                .build();
        TrajectorySequence centerRightSlot = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .waitSeconds(.25)
                .lineToLinearHeading(new Pose2d(42,-5,Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToConstantHeading(new Vector2d(50,-5))
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(50,50))
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.AUTO_HIGH);
                    robot.Arm.longAuto();
                })
                .lineToConstantHeading(new Vector2d(48,50.01))
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(32,69.5))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(34.5,60))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                })
                .lineToConstantHeading(new Vector2d(4,60))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .lineToConstantHeading(new Vector2d(4,75))
                .build();

        TrajectorySequence centerLeftSlot = drive.trajectorySequenceBuilder(newStart)
                .addDisplacementMarker(() -> {
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .waitSeconds(.25)
                .lineToLinearHeading(new Pose2d(42,-5,Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                    robot.Arm.setPosition(armState.medium);
                })
                .lineToConstantHeading(new Vector2d(50,-5))
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(50,50))
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.AUTO_HIGH);
                    robot.Arm.longAuto();
                })
                .lineToConstantHeading(new Vector2d(48,50.01))
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(28,70.5))
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(28,60))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                    robot.Claw.afterTape();
                })
                .lineToConstantHeading(new Vector2d(4,60))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(4,75))
                .build();
        initCam();
        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;
        if (blueDetection.getLocation().equals("LEFT")){
            drive.followTrajectorySequence(left);
        }else if(blueDetection.getLocation().equals("MIDDLE")){
            drive.followTrajectorySequence(centerLeftSlot);
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
        blueDetection = new longBlueObjectDetect(telemetry);

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