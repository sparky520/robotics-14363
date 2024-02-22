package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class bluePaths {

    bluePaths(){

    }
    public TrajectorySequence tape(Robot robot, SampleMecanumDrive drive, Pose2d startPose, String TSEloc, String quadrant){
        robot.Claw.setPosition(armState.close);
        robot.wrist.setPosition(armState.intakingCLAW);
        robot.Arm.intake();
        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.ZERO);
        TrajectorySequence returnSequence;
        if (quadrant.equals("LONG")){
            //LONGLEFT
            returnSequence = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(1.7,() -> {
                        robot.Claw.openLeft();
                    })
                    .lineToLinearHeading(new Pose2d(29,0,Math.toRadians(-270)))
                    .lineToConstantHeading(new Vector2d(29,5))
                    .splineToConstantHeading(new Vector2d(29,0),Math.toRadians(290))
                    .lineToSplineHeading(new Pose2d(7,0,Math.toRadians(270)))
                    .build();
            if (TSEloc.equals("RIGHT")){
                returnSequence = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(1,() -> {
                            robot.Claw.openLeft();
                        })
                        .lineToLinearHeading(new Pose2d(23,-8.5,Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(-90)))
                        .build();
            }else if (TSEloc.equals("MIDDLE")){
                returnSequence = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(1,() -> {
                            robot.Claw.openLeft();
                        })
                        .lineToLinearHeading(new Pose2d(29,1,Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(-90)))
                        .build();
            }
        }else{
            //SHORTLEFT
            returnSequence = drive.trajectorySequenceBuilder(startPose)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                    .addTemporalMarker(1.75,() -> {
                        robot.Claw.openLeft();
                    })
                    .addTemporalMarker(2,() -> {
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.INBETWEEN_SHORT_AND_LONG_AUTO);
                        robot.wrist.setPosition(armState.outtaking);
                        robot.Arm.setPosition(armState.outtaking);
                    })
                    .addTemporalMarker(2.5,() -> {
                        robot.Claw.closeLeft();
                    })
                    .lineToSplineHeading(new Pose2d(30,15,Math.toRadians(-90)))
                    .lineToConstantHeading(new Vector2d(23,30))
                    .waitSeconds(.5)
                    .build();
            if (TSEloc.equals("RIGHT")){
                returnSequence = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(1,() -> {
                            robot.Claw.openLeft();
                        })
                        .addTemporalMarker(2.5,() -> {
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.SHORT_AUTO);
                            robot.wrist.setPosition(armState.outtaking);
                            robot.Arm.setPosition(armState.outtaking);
                        })
                        .lineToSplineHeading(new Pose2d(32,-6,Math.toRadians(-90)))
                        .lineToConstantHeading(new Vector2d(25,30))
                        .build();
            }else if (TSEloc.equals("MIDDLE")){
                returnSequence = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(2,() -> {
                            robot.Claw.openLeft();
                        })
                        .addTemporalMarker(2.5,() -> {
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.SHORT_AUTO);
                            robot.wrist.setPosition(armState.outtaking);
                            robot.Arm.setPosition(armState.outtaking);
                        })
                        .lineToSplineHeading(new Pose2d(41,3,Math.toRadians(-90)))
                        .lineToConstantHeading(new Vector2d(25,30))
                        .build();
            }
        }

        return returnSequence;
    }
    public TrajectorySequence throughTruss(Robot robot, SampleMecanumDrive drive, Pose2d startPose,Double distanceSide, int curCycle){
        TrajectorySequence returnSequence = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(.1,() -> {
                    robot.Arm.half();
                })
                .addTemporalMarker(2,()->{
                    if (curCycle == 0){
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.LONG_AUTO);
                    }else{
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.MEDIUM);
                    }
                })
                .addTemporalMarker(3,() -> {
                    if (curCycle == 0){
                        robot.wrist.setPosition(armState.outtaking);
                    }else{
                        robot.wrist.autoOuttake();
                    }
                    robot.Claw.closeLeft();
                })
                .addTemporalMarker(3.25,() -> {
                    if (curCycle == 0){
                        robot.Arm.setPosition(armState.outtaking);
                    }else{
                        robot.Arm.autoOuttake();
                    }
                })
                .lineToSplineHeading(new Pose2d(5,75,Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(25,105),Math.toRadians(280)).build();
        if (distanceSide < 3 || distanceSide > 6){
            returnSequence = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(.1,() -> {
                        robot.Arm.half();
                    })
                    .addTemporalMarker(2,()->{
                        if (curCycle == 0){
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.LONG_AUTO);
                        }else{
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.MEDIUM);
                        }
                    })
                    .addTemporalMarker(3,() -> {
                        if (curCycle == 0){
                            robot.wrist.setPosition(armState.outtaking);
                        }else{
                            robot.wrist.autoOuttake();
                        }
                        robot.Claw.closeLeft();
                    })
                    .addTemporalMarker(3.25,() -> {
                        if (curCycle == 0){
                            robot.Arm.setPosition(armState.outtaking);
                        }else{
                            robot.Arm.autoOuttake();
                        }
                    })
                    .lineToConstantHeading(new Vector2d(5, startPose.getY()))
                    .lineToSplineHeading(new Pose2d(5,75,Math.toRadians(-90)))
                    .splineToConstantHeading(new Vector2d(25,105),Math.toRadians(280)).build();
        }
        return returnSequence;
    }
    public TrajectorySequence toBoard(Robot robot, SampleMecanumDrive drive, Pose2d startPose,double aprilLoc){
        robot.Claw.closeLeft();
        TrajectorySequence returnSequence = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(aprilLoc,93,Math.toRadians(-90))).build();
        return returnSequence;
    }
    public TrajectorySequence goNextToWall(Robot robot, SampleMecanumDrive drive, Pose2d startPose, String TSEloc, String parkType, String pathType){
        robot.Claw.setPosition(armState.open);
        TrajectorySequence returnSequence;
        if (pathType.equals("2+2")){
            if (TSEloc.equals("RIGHT")){
                returnSequence = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(.5,() -> {
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TOPSTACK);
                        })
                        .addTemporalMarker(1,() -> {
                            robot.wrist.topStack();
                            robot.Arm.topStack();
                        })
                        .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY() - 6))
                        .lineToConstantHeading(new Vector2d(1,62)).build();
            }else{
                returnSequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addTemporalMarker(.5,() -> {
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TOPSTACK);
                        })
                        .addTemporalMarker(1,() -> {
                            robot.wrist.topStack();
                            robot.Arm.topStack();
                        })
                        .lineToConstantHeading(new Vector2d(5,62)).build();
            }
        }else {
            returnSequence = park(robot,drive,startPose,parkType);
        }
        return returnSequence;
    }

    public TrajectorySequence backThroughTruss(Robot robot, SampleMecanumDrive drive, Pose2d startPose,Double distanceSide){
        TrajectorySequence returnSequence = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(5.5,15,Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(25,-13),Math.toRadians(280)).build();
        if (distanceSide < 2 || distanceSide > 6){
            returnSequence = drive.trajectorySequenceBuilder(startPose)
                    .lineToConstantHeading(new Vector2d(4.5, startPose.getY()))
                    .lineToSplineHeading(new Pose2d(4,15,Math.toRadians(-90)))
                    .splineToConstantHeading(new Vector2d(25,-13),Math.toRadians(280)).build();
        }
        return returnSequence;
    }

    public TrajectorySequence goToStack(Robot robot, SampleMecanumDrive drive, Pose2d startPose){
        TrajectorySequence returnSequence = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(21, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(27,0))
                .build();
        return returnSequence;
    }
    public TrajectorySequence intakeStackAndReset(Robot robot, SampleMecanumDrive drive, Pose2d startPose) {
        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.INTAKE_STACK);
        robot.Claw.setPosition(armState.close);
        TrajectorySequence returnSequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(5,29,Math.toRadians(-90)))
                .build();
        return returnSequence;
    }
    public TrajectorySequence park(Robot robot, SampleMecanumDrive drive, Pose2d startPose, String parkType){
        robot.Claw.setPosition(armState.open);
        TrajectorySequence returnSequence = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(.5,()->{
                    robot.Arm.setPosition(armState.medium);
                    robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.ZERO);
                    robot.wrist.setPosition(armState.intakingCLAW);
                })
                .addTemporalMarker(2,() -> {
                    robot.Arm.setPosition(armState.low);
                })
                .lineToConstantHeading(new Vector2d(20,84))
                .lineToLinearHeading(new Pose2d(0,84,Math.toRadians(0))).build();
        if (parkType.equals("CENTER")){
            returnSequence = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(.5,()->{
                        robot.Arm.setPosition(armState.medium);
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.ZERO);
                        robot.wrist.setPosition(armState.intakingCLAW);
                    })
                    .addTemporalMarker(2,() -> {
                        robot.Arm.setPosition(armState.low);
                    })
                    .lineToConstantHeading(new Vector2d(20,84))
                    .lineToLinearHeading(new Pose2d(50,84,Math.toRadians(0))).build();
        }
        return returnSequence;
    }
}
