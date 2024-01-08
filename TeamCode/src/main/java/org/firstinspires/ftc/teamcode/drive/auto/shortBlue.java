package org.firstinspires.ftc.teamcode.drive.auto;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.SubSystems.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.arm;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class shortBlue extends LinearOpMode
{
    Robot robot;
    OpenCvCamera roboCam;
    String webcamName;
    objdetect_blue blueTSE;
    @Override
    public void runOpMode() {
        webcamName = "Webcam 1";
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        roboCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        objdetect_blue detector = new objdetect_blue(telemetry);
        roboCam.setPipeline(detector);
        roboCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                roboCam.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("error opening cam");
                telemetry.update();
            }
        });

        waitForStart();
        if(blueTSE.getLocation() == objdetect_blue.Location.MIDDLE){

            robot = new Robot(hardwareMap, telemetry);
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Pose2d newStart = new Pose2d();
            TrajectorySequence center = drive.trajectorySequenceBuilder(newStart)
                    .addDisplacementMarker(() -> {
                        robot.Claw.setPosition(armState.intakingCLAW);
                    })
                    .lineToLinearHeading(new Pose2d(36.25,0,Math.toRadians(-90)))
                    .addDisplacementMarker(() -> {
                        robot.Claw.setTape();
                        robot.Arm.setPosition(armState.medium);
                    })
                    .lineToConstantHeading(new Vector2d(36,9.01))
                    .waitSeconds(.25)
                    .addDisplacementMarker(() -> {
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                        robot.Arm.setPosition(armState.high);
                    })
                    .lineToConstantHeading(new Vector2d(31,9.02))
                    .waitSeconds(1)
                    .lineToConstantHeading(new Vector2d(28,30))
                    .addDisplacementMarker(() -> {
                        robot.Claw.dropBoard();
                    })
                    .lineToConstantHeading(new Vector2d(20,18))
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
                    .lineToConstantHeading(new Vector2d(0,30))
                    .build();
        /*
                .lineToLinearHeading(new Pose2d(13,15,Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                    robot.Arm.setPosition(armState.high);
                })
                .lineToConstantHeading(new Vector2d(13,14))
                //.waitSeconds(2)
                .lineToConstantHeading(new Vector2d(28,29.5))//this one
                //.waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.Claw.dropBoard();
                })
                .lineToConstantHeading(new Vector2d(25.5,18))
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.medium);
                    robot.Claw.setPosition(armState.intakingCLAW);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                })
                .lineToConstantHeading(new Vector2d(0,23))
                .addDisplacementMarker(() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .lineToConstantHeading(new Vector2d(0,30))
                .build();
                */

            waitForStart();

            if(isStopRequested()) return;
            drive.followTrajectorySequence(center);
        }

        if(blueTSE.getLocation() == objdetect_blue.Location.LEFT){
            robot = new Robot(hardwareMap, telemetry);
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Pose2d newStart = new Pose2d();
            TrajectorySequence left = drive.trajectorySequenceBuilder(newStart)
                    .addDisplacementMarker(() -> {
                        robot.Claw.setPosition(armState.intakingCLAW);
                    })
                    .lineToLinearHeading(new Pose2d(31,9,Math.toRadians(-90)))
                    .addDisplacementMarker(() -> {
                        robot.Claw.setTape();
                        robot.Arm.setPosition(armState.medium);
                    })
                    .lineToConstantHeading(new Vector2d(31,9.01))
                    .waitSeconds(.25)
                    .addDisplacementMarker(() -> {
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                        robot.Arm.setPosition(armState.high);
                    })
                    .lineToConstantHeading(new Vector2d(31,9.02))
                    .waitSeconds(1)
                    .lineToConstantHeading(new Vector2d(23,29.25))
                    .addDisplacementMarker(() -> {
                        robot.Claw.dropBoard();
                    })
                    .lineToConstantHeading(new Vector2d(20,18))
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
                    .lineToConstantHeading(new Vector2d(0,30))
                    .build();
            waitForStart();

            if(isStopRequested()) return;
            drive.followTrajectorySequence(left);

        }
        if(blueTSE.getLocation()== objdetect_blue.Location.RIGHT){

            robot = new Robot(hardwareMap, telemetry);
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Pose2d newStart = new Pose2d();
            TrajectorySequence right = drive.trajectorySequenceBuilder(newStart)
                    .addDisplacementMarker(() -> {
                        robot.Claw.setPosition(armState.intakingCLAW);
                    })
                    .lineToLinearHeading(new Pose2d(28,6,Math.toRadians(-90)))
                    .lineToConstantHeading(new Vector2d(28,-11))
                    .addDisplacementMarker(() -> {
                        robot.Claw.setTape();
                        robot.Arm.setPosition(armState.medium);
                    })
                    .lineToConstantHeading(new Vector2d(28,-10.99))
                    .waitSeconds(.25)
                    .addDisplacementMarker(() -> {
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
                        robot.Arm.setPosition(armState.high);
                        robot.Claw.setPosition(armState.intakingCLAW);
                    })
                    //.lineToConstantHeading(new Vector2d(31,9.02))
                    //.waitSeconds(1)
                    .lineToConstantHeading(new Vector2d(33.75,30.75))
                    .addDisplacementMarker(() -> {
                        robot.Claw.dropBoard();
                    })
                    .lineToConstantHeading(new Vector2d(33,18))
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
                    .lineToConstantHeading(new Vector2d(0,30))
                    .build();
            waitForStart();

            if(isStopRequested()) return;
            drive.followTrajectorySequence(right);
        }
    }
}