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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class shortBlueCenter extends LinearOpMode {
    boolean leftTag = false;
    double boardOffset = 4;

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
    Pose2d start = new Pose2d(0, 0, Math.toRadians(180));
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
        drive.setPoseEstimate(start);
        robot = new Robot(hardwareMap, telemetry);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distancesensor");
        Trajectory tape = drive.trajectoryBuilder(start)
                .addDisplacementMarker(() -> {
                    //robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                    //robot.Arm.setPosition(armState.low);
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToLinearHeading(new Pose2d(34,7, Math.toRadians(-70)))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                }).build();
        TrajectorySequence boardStack1 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,1)).build();
        TrajectorySequence boardStack2 = drive.trajectorySequenceBuilder(boardStack1.end()).lineToConstantHeading(new Vector2d(0,2)).build();
        TrajectorySequence boardStack3 = drive.trajectorySequenceBuilder(boardStack2.end()).lineToConstantHeading(new Vector2d(0,3)).build();

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
                        telemetry.addLine("  " + tagOfInterest.id);
                        telemetry.addLine(toBoardEnd.getX()-(100*tagOfInterest.pose.x/6/1.41) + "");
                        boardX = toBoardEnd.getX()-20 - (100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY()- 6 +(100*tagOfInterest.pose.z/6);
                        telemetry.addLine(boardX + "  " + boardY );
                        telemetry.update();
                        boardStack1 = drive.trajectorySequenceBuilder(tape.end())
                                .lineToLinearHeading(new Pose2d(boardX,boardY,Math.toRadians(-90)))
                                .lineToConstantHeading(new Vector2d(25,boardY-50), SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .lineToConstantHeading(new Vector2d(26.5,boardY+9)).build();
                        caseTagFound = true;
                    }
                    if (!drive.isBusy() && caseTagFound == true){
                        drive.followTrajectorySequenceAsync(boardStack1);
                        caseTagFound = false;
                        currentState = state.stack1;
                    }
                case stack1:
                    if (tagOfInterest != null && caseTagFound == false) {
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX() - 2 - (100*tagOfInterest.pose.x / 6 / 1.41);
                        boardY = toBoardEnd.getY() - boardOffset + (100*tagOfInterest.pose.z / 6);
                        boardStack2 = drive.trajectorySequenceBuilder(boardStack1.end())
                                .lineToConstantHeading(new Vector2d(boardX,boardY))
                                .lineToConstantHeading(new Vector2d(26,boardY-50),SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .lineToConstantHeading(new Vector2d(26,boardY+9)).build();
                    }
                    if (!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(boardStack2);
                        caseTagFound = false;
                        currentState = state.stack2;
                    }
                case stack2:
                    if (tagOfInterest != null && caseTagFound == false) {
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX() - 2 -(100*tagOfInterest.pose.x / 6 / 1.41);
                        boardY = toBoardEnd.getY() + (100*tagOfInterest.pose.z / 6);
                        boardStack3 = drive.trajectorySequenceBuilder(boardStack2.end())
                                .lineToConstantHeading(new Vector2d(boardX,boardY-boardOffset)).build();
                    }
                    if (!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(boardStack3);
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

    void detectTags() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if (currentDetections.size() != 0) {
            tagFound = false;
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == middle || tag.id == middle || tag.id == middle) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }
        }
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