package org.firstinspires.ftc.teamcode.drive.autoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class shortBlueCenterSpline extends LinearOpMode {
    double boardOffset = 9;

    int cycle = 0;

    enum state {
        tape,stack1,stack2,stack3,IDLE
    }

    boolean tagFound = false;
    boolean caseTagFound = false;
    Robot robot;
    state currentState = state.IDLE;
    ElapsedTime timer = new ElapsedTime();
    DistanceSensor distanceSensor;
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    double boardX, boardY;
    aprilTagDetection aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    shortBlueObjectDetectTest blueDetect;
    int left = 4;
    int middle = 5;
    int right = 6;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        robot = new Robot(hardwareMap, telemetry);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distancesensor");
        Trajectory tape = drive.trajectoryBuilder(new Pose2d(0,0))
                .addDisplacementMarker(() -> {
                    //robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                    //robot.Arm.setPosition(armState.low);
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .splineToLinearHeading(new Pose2d(-37,-7, Math.toRadians(90)),0)
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                }).build();
        Trajectory boardStack1 = drive.trajectoryBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,1)).build();
        Trajectory boardStack2 = drive.trajectoryBuilder(boardStack1.end()).lineToConstantHeading(new Vector2d(0,2)).build();
        Trajectory boardStack3 = drive.trajectoryBuilder(boardStack2.end()).lineToConstantHeading(new Vector2d(0,3)).build();

        initColorDetection();

        /* Actually do something useful */
        waitForStart();
        camera.stopStreaming();
        initAprilTagDetect();

        if (isStopRequested()) return;
        currentState = state.tape;
        robot.Claw.setPosition(armState.intakingCLAW);
        drive.followTrajectoryAsync(tape);
        timer.reset();
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case tape:
                    if (tagOfInterest != null && caseTagFound == false){
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX()-(100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY()-(100*tagOfInterest.pose.z/6);
                        telemetry.addLine(tagOfInterest.pose.x/6/1.41 + " cam " + tagOfInterest.pose.z/6);
                        telemetry.addLine(toBoardEnd.getX() + " pose "+ toBoardEnd.getY());
                        telemetry.addData("Traj", boardY);
                        telemetry.update();
                        boardStack1 = drive.trajectoryBuilder(tape.end())
                                .addDisplacementMarker(() -> {
                                    robot.Arm.setPosition(armState.outtaking);
                                    robot.Claw.setPosition(armState.intakingCLAW);
                                })
                                .splineTo(new Vector2d(boardX,boardY), Math.toRadians(90))
                                .addDisplacementMarker(() -> {
                                    robot.Claw.setPosition(armState.outtaking);
                                })
                                .splineTo(new Vector2d(-24,-5),Math.toRadians(90))
                                .addDisplacementMarker(() -> {
                                    toStack(1);
                                })
                                .splineTo(new Vector2d(-26.5,48),Math.toRadians(90))
                                .addDisplacementMarker(() -> {
                                    robot.Claw.setPosition(armState.intakingCLAW);
                                })
                                .splineTo(new Vector2d(-24,-20),Math.toRadians(90))
                                .addDisplacementMarker(() -> {
                                    robot.Arm.setPosition(armState.outtaking);
                                }).build();
                        caseTagFound = true;
                    }
                    if (!drive.isBusy()){
                        drive.followTrajectoryAsync(boardStack1);
                        caseTagFound = false;
                        currentState = state.stack1;
                    }
                case stack1:
                    if (tagOfInterest != null && caseTagFound == false) {
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX() - (100*tagOfInterest.pose.x / 6 / 1.41);
                        boardY = toBoardEnd.getY() - (100*tagOfInterest.pose.z / 6);
                        boardStack2 = drive.trajectoryBuilder(boardStack1.end())
                                .splineToConstantHeading(new Vector2d(boardX,boardY), Math.toRadians(90))
                                .addDisplacementMarker(() -> {
                                    robot.Claw.setPosition(armState.outtaking);
                                })
                                .splineToConstantHeading(new Vector2d(-24,-35),Math.toRadians(90))
                                .addDisplacementMarker(() -> {
                                    toStack(2);
                                })
                                .splineToConstantHeading(new Vector2d(-26.5,20),Math.toRadians(90))
                                .addDisplacementMarker(() -> {
                                    robot.Claw.setPosition(armState.intakingCLAW);
                                })
                                .splineToConstantHeading(new Vector2d(-24,-50),Math.toRadians(90))
                                .addDisplacementMarker(() -> {
                                    robot.Arm.setPosition(armState.outtaking);
                                }).build();
                    }
                    if (!drive.isBusy()){
                        drive.followTrajectoryAsync(boardStack2);
                        caseTagFound = false;
                        currentState = state.stack2;
                    }
                case stack2:
                    if (tagOfInterest != null && caseTagFound == false) {
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX() - (100*tagOfInterest.pose.x / 6 / 1.41);
                        boardY = toBoardEnd.getY() - (100*tagOfInterest.pose.z / 6);
                        boardStack3 = drive.trajectoryBuilder(boardStack1.end())
                                .splineToConstantHeading(new Vector2d(boardX,boardY), Math.toRadians(90))
                                .addDisplacementMarker(() -> {
                                    robot.Claw.setPosition(armState.outtaking);
                                }).build();
                    }
                    if (!drive.isBusy()){
                        drive.followTrajectoryAsync(boardStack3);
                        caseTagFound = false;
                        currentState = state.IDLE;
                    }
                case stack3:
                    if (!drive.isBusy()){
                        currentState = state.IDLE;
                    }
            }

            drive.update();
            detectTags();
        }
    }

    public void toStack(int cycle) {
        ElapsedTime timer1 = new ElapsedTime();
        robot.Arm.setPosition(armState.medium);
        while (timer1.seconds() < .5) {
            continue;
        }
        if (cycle == 1){
            robot.Arm.topStack();
        }else if (cycle == 2){
            robot.Arm.topStack();
        }

    }

    void detectTags() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if(currentDetections.size() != 0)
        {
            tagFound = false;
            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == middle || tag.id == right || tag.id == left)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }
            if(tagFound)
            {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        }
        else
        {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null)
            {
                telemetry.addLine("(The tag has never been seen)");
            }
            else
            {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

        telemetry.update();
        sleep(20);

    }
    private void initColorDetection() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        blueDetect = new shortBlueObjectDetectTest(telemetry);

        camera.setPipeline(blueDetect);
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
    void initAprilTagDetect() {
        aprilTagDetectionPipeline = new aprilTagDetection(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.setMsTransmissionInterval(50);
    }

    void tagToTelemetry(AprilTagDetection detection) {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x / 6 / 1.41));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z / 6));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}