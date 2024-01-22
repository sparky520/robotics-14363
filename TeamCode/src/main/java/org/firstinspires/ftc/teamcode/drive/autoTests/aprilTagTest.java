/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.autoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.autoExecutable.shortBlueObjectDetect;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@TeleOp
public class aprilTagTest extends LinearOpMode{
        double distanceSums = 0;
        int totalDistanceRecordings = 0;
        double boardOffset = 9;

    int cycle = 0;
    enum state{
        toTape, wait1, toBoard, wait3, moveAwayFromBoard, retractArm, toStack, toBoardFromStack1, toBoardFromStack2, IDLE, toBoardFromStack_,toBoardFromStack2_,
    }
    boolean tagFound = false;
    Trajectory tape, board, sensorToBoard, moveAwayFromBoard, toStack, toBoardFromStack, toStack2, toBoardFromStack_, toBoardFromStack2_, toBoardFromStack2, moveAwayFromBoard1, moveAwayFromBoard2;
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
    public void runOpMode()
    {
        initColorDetection();

        /* Actually do something useful */
        waitForStart();
        camera.stopStreaming();
        initAprilTagDetect();

        while (opModeIsActive() && !isStopRequested()){detectTags();}

        initPaths(blueDetect.getLocation());
        currentState = state.toTape;
        robot.Claw.setPosition(armState.intakingCLAW);
        drive.followTrajectoryAsync(tape);
        while (opModeIsActive() && !isStopRequested()){
            if (blueDetect.getLocation().equals("LEFT")){
                detectTags();
            }else if(blueDetect.getLocation().equals("MIDDLE")){
                /*
                switch(currentState){
                    case toTape:
                        if (!drive.isBusy()){
                            currentState = state.wait1;
                            robot.Claw.setTape();
                            timer.reset();
                        }
                        break;
                    case wait1:
                        if (timer.seconds() > .3){
                            robot.Arm.setPosition(armState.medium);
                            outtakeAfterMedium();
                            drive.followTrajectoryAsync(board);
                            currentState = state.toBoard;
                        }
                        break;
                    case toBoard:
                        if (!drive.isBusy()){
                            ElapsedTime timer2 = new ElapsedTime();
                            timer2.reset();
                            double distance = distanceSensor.getDistance(DistanceUnit.INCH);
                            while (timer2.seconds()<.6){
                                if (distance < 20){
                                    totalDistanceRecordings +=1;
                                    distanceSums += distanceSensor.getDistance(DistanceUnit.INCH);
                                }
                            }
                            if (timer2.seconds() > .6){
                                if (cycle == 0){
                                    sensorToBoard = createPathToBoard(board.end());
                                }else if (cycle == 1){
                                    sensorToBoard = createPathToBoard(toBoardFromStack.end());
                                }else if (cycle == 2){
                                    sensorToBoard = createPathToBoard(toBoardFromStack2.end());
                                }
                                drive.followTrajectoryAsync(sensorToBoard);
                                currentState = state.wait3;
                            }
                        }
                    case wait3:
                        if (!drive.isBusy()){
                            robot.Claw.dropBoard();
                            currentState = state.moveAwayFromBoard;
                            timer.reset();
                        }
                        break;
                    case moveAwayFromBoard:
                        if (timer.seconds() > .3) {
                            if (cycle == 0){
                                drive.followTrajectoryAsync(moveAwayFromBoard);
                            }else if (cycle == 1){
                                drive.followTrajectoryAsync(moveAwayFromBoard1);
                            }else if (cycle == 2){
                                drive.followTrajectoryAsync(moveAwayFromBoard2);
                            }
                            currentState = state.retractArm;
                            timer.reset();
                        }
                        break;
                    case retractArm:
                        if (timer.seconds() > .5){
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                            robot.Arm.setPosition(armState.medium);
                            robot.Claw.setPosition(armState.intakingCLAW);
                            currentState = state.toStack;
                        }
                        break;
                    case toStack:

                        if (!drive .isBusy()){
                            cycle+=1;
                            if (cycle == 1){
                                drive.followTrajectoryAsync(toStack);
                                robot.Arm.topStack();
                                robot.Claw.setPosition(armState.outtaking);
                                currentState = state.toBoardFromStack1;
                            }else if (cycle == 2){
                                drive.followTrajectoryAsync(toStack2);
                                robot.Arm.topStack();
                                robot.Claw.setPosition(armState.outtaking);
                                currentState = state.toBoardFromStack2;
                            }else{
                                currentState = state.IDLE;
                            }
                        }
                    case toBoardFromStack1:
                        if (!drive.isBusy()){
                            drive.followTrajectoryAsync(toBoardFromStack_);
                            currentState = state.toBoardFromStack_;
                        }
                    case toBoardFromStack_:
                        if (!drive.isBusy()){
                            drive.followTrajectoryAsync(toBoardFromStack);
                            robot.Arm.setPosition(armState.medium);
                            outtakeAfterMedium();
                            currentState = state.toBoard;
                        }
                    case toBoardFromStack2:
                        if (!drive.isBusy()){;
                            drive.followTrajectoryAsync(toBoardFromStack2_);
                            currentState = state.toBoardFromStack2_;
                        }
                    case toBoardFromStack2_:
                        if (!drive.isBusy()){
                            drive.followTrajectoryAsync(toBoardFromStack2);
                            robot.Arm.setPosition(armState.medium);
                            outtakeAfterMedium();
                            currentState = state.toBoard;
                        }
                    case IDLE:
                        break;
                }

                drive.update();
                detectTags();
                */
                switch(currentState){
                    case toTape:
                        if (!drive.isBusy()){
                            currentState = state.wait1;
                            robot.Claw.setTape();
                            detectTags();
                            Pose2d end = tape.end();
                            board = drive.trajectoryBuilder(end).lineToConstantHeading(new Vector2d(end.getX()-(tagOfInterest.pose.x/6/1.41), end.getY()+(tagOfInterest.pose.y/6))).build();
                            timer.reset();
                        }
                        break;
                    case wait1:
                        if (timer.seconds() > .3){
                            robot.Arm.setPosition(armState.medium);
                            outtakeAfterMedium();
                            drive.followTrajectoryAsync(board);
                            currentState = state.toBoard;
                        }
                        break;
                    case toBoard:
                        if (!drive.isBusy()){
                            ElapsedTime timer2 = new ElapsedTime();
                            timer2.reset();
                            double distance = distanceSensor.getDistance(DistanceUnit.INCH);
                            while (timer2.seconds()<.6){
                                if (distance < 20){
                                    totalDistanceRecordings +=1;
                                    distanceSums += distanceSensor.getDistance(DistanceUnit.INCH);
                                }
                            }
                            if (timer2.seconds() > .6){
                                if (cycle == 0){
                                    sensorToBoard = createPathToBoard(board.end());
                                }else if (cycle == 1){
                                    sensorToBoard = createPathToBoard(toBoardFromStack.end());
                                }else if (cycle == 2){
                                    sensorToBoard = createPathToBoard(toBoardFromStack2.end());
                                }
                                drive.followTrajectoryAsync(sensorToBoard);
                                currentState = state.wait3;
                            }
                        }
                    case wait3:
                        if (!drive.isBusy()){
                            robot.Claw.dropBoard();
                            currentState = state.moveAwayFromBoard;
                            timer.reset();
                        }
                        break;
                    case moveAwayFromBoard:
                        if (timer.seconds() > .3) {
                            if (cycle == 0){
                                drive.followTrajectoryAsync(moveAwayFromBoard);
                            }else if (cycle == 1){
                                drive.followTrajectoryAsync(moveAwayFromBoard1);
                            }else if (cycle == 2){
                                drive.followTrajectoryAsync(moveAwayFromBoard2);
                            }
                            currentState = state.retractArm;
                            timer.reset();
                        }
                        break;
                    case retractArm:
                        if (timer.seconds() > .5){
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                            robot.Arm.setPosition(armState.medium);
                            robot.Claw.setPosition(armState.intakingCLAW);
                            currentState = state.toStack;
                        }
                        break;
                    case toStack:

                        if (!drive .isBusy()){
                            cycle+=1;
                            tagFound = false;
                            if (cycle == 1){
                                drive.followTrajectoryAsync(toStack);
                                robot.Arm.topStack();
                                robot.Claw.setPosition(armState.outtaking);
                                currentState = state.toBoardFromStack1;
                            }else if (cycle == 2){
                                drive.followTrajectoryAsync(toStack2);
                                robot.Arm.topStack();
                                robot.Claw.setPosition(armState.outtaking);
                                currentState = state.toBoardFromStack2;
                            }else{
                                currentState = state.IDLE;
                            }
                        }
                    case toBoardFromStack1:
                        if (tagOfInterest != null){
                            Pose2d toBoardEnd = drive.getPoseEstimate();
                            boardX = toBoardEnd.getX()-(tagOfInterest.pose.x/6/1.41);
                            boardY = toBoardEnd.getY()+(tagOfInterest.pose.y/6);
                        }
                        if (!drive.isBusy()){
                            drive.followTrajectoryAsync(toBoardFromStack_);
                            currentState = state.toBoardFromStack_;
                        }
                    case toBoardFromStack_:
                        if (!drive.isBusy()){
                            drive.followTrajectoryAsync(toBoardFromStack);
                            robot.Arm.setPosition(armState.medium);
                            outtakeAfterMedium();
                            currentState = state.toBoard;
                        }
                    case toBoardFromStack2:
                        if (!drive.isBusy()){;
                            drive.followTrajectoryAsync(toBoardFromStack2_);
                            currentState = state.toBoardFromStack2_;
                        }
                    case toBoardFromStack2_:
                        if (!drive.isBusy()){
                            drive.followTrajectoryAsync(toBoardFromStack2);
                            robot.Arm.setPosition(armState.medium);
                            outtakeAfterMedium();
                            currentState = state.toBoard;
                        }
                    case IDLE:
                        break;
                }

                drive.update();
                detectTags();
            }else if(blueDetect.getLocation().equals("RIGHT")){

            }
        }
    }

    void initPaths(String tapeLocation){
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);
        robot = new Robot(hardwareMap, telemetry);
        if (tapeLocation.equals("LEFT")){

        }else if (tapeLocation.equals("MIDDLE")){
            tape = drive.trajectoryBuilder(start).lineToLinearHeading(new Pose2d(37,7,Math.toRadians(-90))).build();
            board = drive.trajectoryBuilder(tape.end()).lineToConstantHeading(new Vector2d(24,32)).build();
             sensorToBoard = drive.trajectoryBuilder(tape.end()).lineToConstantHeading(new Vector2d(24,32)).build();
             moveAwayFromBoard = drive.trajectoryBuilder(sensorToBoard.end()).lineToConstantHeading(new Vector2d(24,5)).build();
             toStack = drive.trajectoryBuilder(moveAwayFromBoard.end()).lineToConstantHeading(new Vector2d(26.5,-48)).build();
             toBoardFromStack_ = drive.trajectoryBuilder(toStack.end()).lineToConstantHeading(new Vector2d(24,20)).build();
             toBoardFromStack = drive.trajectoryBuilder(toBoardFromStack_.end()).lineToConstantHeading(new Vector2d(24,50)).build();
             moveAwayFromBoard1 = drive.trajectoryBuilder(toBoardFromStack.end()).lineToConstantHeading(new Vector2d(24,35)).build();
             toStack2 = drive.trajectoryBuilder(toBoardFromStack.end()).lineToConstantHeading(new Vector2d(26.5,-20)).build();
             toBoardFromStack2_ = drive.trajectoryBuilder(toStack2.end()).lineToConstantHeading(new Vector2d(24,50)).build();
             toBoardFromStack2 = drive.trajectoryBuilder(toBoardFromStack2_.end()).lineToConstantHeading(new Vector2d(24,80)).build();
             moveAwayFromBoard2 = drive.trajectoryBuilder(toBoardFromStack2.end()).lineToConstantHeading(new Vector2d(24,65)).build();
        }
        else if (tapeLocation.equals("RIGHT")){

        }
    }
    void detectTags(){
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
    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x/6/1.41));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z/6));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
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
    public void outtakeAfterMedium(){
        ElapsedTime timer1 = new ElapsedTime();
        while (timer1.seconds() < .6){
            continue;
        }
        robot.Arm.setPosition(armState.high);
        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
    }
    public Trajectory createPathToBoard(Pose2d endPose){
        Trajectory createdPath;
        double averageDistance = distanceSums/totalDistanceRecordings;
        double finalDistance = averageDistance - boardOffset;
        createdPath = drive.trajectoryBuilder(endPose).forward(-finalDistance).build();
        telemetry.addData("final", finalDistance);
        telemetry.update();
        return createdPath;
    }
}