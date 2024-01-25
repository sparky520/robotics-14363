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
public class shortBlueTest extends LinearOpMode {
    boolean leftTag = false;
    double boardOffset = 4;

    int cycle = 0;

    enum state {
        tape,board,stack1,board2,stack2,board3,stack3,IDLE
    }

    boolean tagFound = false;
    boolean caseTagFound = false;
    Robot robot;
    state currentState = state.IDLE;
    ElapsedTime timer = new ElapsedTime();
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
    int location = 2;
    Pose2d boardPose = new Pose2d(26,43,Math.toRadians(-90));
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);
        robot = new Robot(hardwareMap, telemetry);
        Trajectory tape = drive.trajectoryBuilder(start)
                .addDisplacementMarker(() -> {
                    //robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                    //robot.Arm.setPosition(armState.low);
                    robot.Claw.setPosition(armState.intakingCLAW);
                })
                .lineToLinearHeading(new Pose2d(33,7, Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    robot.Claw.setTape();
                }).build();
        TrajectorySequence boardStack1 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,1)).build();
        TrajectorySequence boardstack1_5 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,1.5)).build();
        TrajectorySequence boardStack2 = drive.trajectorySequenceBuilder(new Pose2d(50,35)).lineToConstantHeading(new Vector2d(0,2)).build();
        TrajectorySequence boardstack2_5 = drive.trajectorySequenceBuilder(tape.end()).lineToConstantHeading(new Vector2d(0,2.5)).build();
        TrajectorySequence boardStack3 = drive.trajectorySequenceBuilder(boardStack2.end()).lineToConstantHeading(new Vector2d(0,3)).build();

        initColorDetection();
        /* Actually do something useful */
        waitForStart();
        camera.stopStreaming();
        initAprilTagDetect();

        if (isStopRequested()) return;
        currentState = state.board;
        robot.Claw.setPosition(armState.intakingCLAW);
        drive.followTrajectoryAsync(tape);
        timer.reset();
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case board:
                    if (tagOfInterest != null && caseTagFound == false){
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX()-10 - (100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY()- 10 +(100*tagOfInterest.pose.z/6);
                        telemetry.addLine(boardX + "  " + boardY );
                        telemetry.update();
                        boardStack1 = drive.trajectorySequenceBuilder(tape.end())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(boardX,boardY))
                                .build();
                        caseTagFound = true;
                    }
                    if (!drive.isBusy() && caseTagFound == true){
                        location = 3;
                        drive.followTrajectorySequenceAsync(boardStack1);
                        caseTagFound = false;
                        currentState = state.stack1;
                        tagOfInterest = null;
                    }
                case stack1:
                    if (!drive.isBusy()){
                        boardstack1_5 = drive.trajectorySequenceBuilder(boardPose)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .waitSeconds(.2)
                                .splineToConstantHeading(new Vector2d(50,12), Math.toRadians(280))
                                .splineToConstantHeading(new Vector2d(49,-25),Math.toRadians(280))
                                .waitSeconds(.2)
                                .lineToConstantHeading(new Vector2d(50,35))
                                .build();
                        telemetry.addLine(drive.getPoseEstimate().getX() + "  " + drive.getPoseEstimate().getY());
                        drive.setPoseEstimate(boardPose);
                        telemetry.addLine(drive.getPoseEstimate().getX() + "  " + drive.getPoseEstimate().getY());
                        telemetry.update();
                        drive.followTrajectorySequenceAsync(boardstack1_5);
                        currentState = state.board2;
                    }
                case board2:
                    if (tagOfInterest != null && caseTagFound == false){
                        Pose2d last = boardstack1_5.end();
                        Pose2d realLast = new Pose2d(50,35);
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX()-10 - (100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY()- 10 +(100*tagOfInterest.pose.z/6);
                        telemetry.addLine(boardstack1_5.end().getX() + "  " + boardstack1_5.end().getY());
                        telemetry.addLine(drive.getPoseEstimate().getX() + "  " + drive.getPoseEstimate().getY());
                        telemetry.update();
                        boardStack2 = drive.trajectorySequenceBuilder(realLast)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(boardX - (realLast.getX()- last.getX()),boardY + realLast.getY() - last.getY()))
                                .build();
                        caseTagFound = true;
                    }
                    if (!drive.isBusy() && caseTagFound == true){
                        location = 3;
                        drive.followTrajectorySequenceAsync(boardStack2);
                        caseTagFound = false;
                        currentState = state.stack2;
                        tagOfInterest = null;
                    }
                case stack2:
                    if (!drive.isBusy()){
                        boardstack2_5 = drive.trajectorySequenceBuilder(boardPose)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .waitSeconds(.2)
                                .splineToConstantHeading(new Vector2d(50,12), Math.toRadians(280))
                                .splineToConstantHeading(new Vector2d(49,-25),Math.toRadians(280))
                                .waitSeconds(.2)
                                .lineToConstantHeading(new Vector2d(50,35))
                                .build();
                        telemetry.addLine(drive.getPoseEstimate().getX() + "  " + drive.getPoseEstimate().getY());
                        drive.setPoseEstimate(boardPose);
                        telemetry.addLine(drive.getPoseEstimate().getX() + "  " + drive.getPoseEstimate().getY());
                        telemetry.update();
                        drive.followTrajectorySequenceAsync(boardstack1_5);
                        currentState = state.board3;
                    }
                case board3:
                    if (tagOfInterest != null && caseTagFound == false){
                        Pose2d toBoardEnd = drive.getPoseEstimate();
                        boardX = toBoardEnd.getX()-10 - (100*tagOfInterest.pose.x/6/1.41);
                        boardY = toBoardEnd.getY()- 10 +(100*tagOfInterest.pose.z/6);
                        telemetry.addLine(boardX + " s " + boardY );
                        telemetry.update();
                        boardStack1 = drive.trajectorySequenceBuilder(boardstack2_5.end())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                                .lineToConstantHeading(new Vector2d(boardX,boardY))
                                .build();
                        caseTagFound = true;
                    }
                    if (!drive.isBusy() && caseTagFound == true){
                        location = 3;
                        drive.followTrajectorySequenceAsync(boardStack1);
                        caseTagFound = false;
                        currentState = state.stack3;
                        tagOfInterest = null;
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
                if (tag.id == location) {
                    telemetry.addLine("FOUND");
                    telemetry.update();
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
    void initAprilTagDetect(){
        aprilTagDetectionPipeline = new aprilTagDetection(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        telemetry.setMsTransmissionInterval(50);
    }
}