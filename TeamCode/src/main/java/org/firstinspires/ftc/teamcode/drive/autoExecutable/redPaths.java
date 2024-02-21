package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class redPaths {

    redPaths(){

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
                    .addTemporalMarker(1,() -> {
                        robot.Claw.openLeft();
                    })
                    .lineToLinearHeading(new Pose2d(25,4,Math.toRadians(0)))
                    .lineToSplineHeading(new Pose2d(-1,0,Math.toRadians(90)))
                    .turn(Math.toRadians(-10))
                    .build();
            if (TSEloc.equals("RIGHT")){
                returnSequence = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(1.7,() -> {
                            robot.Claw.openLeft();
                        })
                        .lineToLinearHeading(new Pose2d(28,0,Math.toRadians(-90)))
                        .lineToConstantHeading(new Vector2d(28,-7))
                        .splineToConstantHeading(new Vector2d(28,1),Math.toRadians(270))
                        .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(90)))
                        .turn(Math.toRadians(-10))
                        .build();
            }else if (TSEloc.equals("MIDDLE")){
                returnSequence = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(1,() -> {
                            robot.Claw.openLeft();
                        })
                        .lineToLinearHeading(new Pose2d(32,0,Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(-1,0,Math.toRadians(90)))
                        .turn(Math.toRadians(-10))
                        .build();
            }
        }else{
            //SHORTLEFT
            returnSequence = drive.trajectorySequenceBuilder(startPose)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
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
                    .lineToLinearHeading(new Pose2d(26,5,Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(23,-23,Math.toRadians(90)))
                    .build();
            if (TSEloc.equals("RIGHT")){
                returnSequence = drive.trajectorySequenceBuilder(startPose)
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                        .addTemporalMarker(1.5,() -> {
                            robot.Claw.openLeft();
                        })
                        .addTemporalMarker(2,() -> {
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.SHORT_AUTO);
                            robot.wrist.setPosition(armState.outtaking);
                            robot.Arm.setPosition(armState.outtaking);
                        })
                        .lineToLinearHeading(new Pose2d(32,-15,Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(23,-23,Math.toRadians(90)))
                        .build();
            }else if (TSEloc.equals("MIDDLE")){
                returnSequence = drive.trajectorySequenceBuilder(startPose)
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                        .addTemporalMarker(2,() -> {
                            robot.Claw.openLeft();
                        })
                        .addTemporalMarker(2.5,() -> {
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.SHORT_AUTO);
                            robot.wrist.setPosition(armState.outtaking);
                            robot.Arm.setPosition(armState.outtaking);
                        })
                        .lineToLinearHeading(new Pose2d(42,-3,Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(23,-23,Math.toRadians(90)))
                        .build();
            }
        }
        return returnSequence;
    }
    public TrajectorySequence throughTruss(Robot robot, SampleMecanumDrive drive, Pose2d startPose,Double distanceSide, int curCycle){
        robot.Claw.closeLeft();
        TrajectorySequence returnSequence = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(.1,() -> {
                    robot.Arm.half();
                })
                .addTemporalMarker(2,()->{
                    if (curCycle == 0){
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.LONG_AUTO);
                    }else{
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TWO_PLUS_TWO_OUTTAKE);
                    }
                })
                .addTemporalMarker(2,() -> {
                    if (curCycle == 0){
                        robot.Arm.setPosition(armState.outtaking);
                        robot.wrist.setPosition(armState.outtaking);
                    }else{
                        robot.Arm.autoOuttake();
                        robot.wrist.autoOuttake();
                    }
                })
                .lineToSplineHeading(new Pose2d(6.5,-75,Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(25,-105),Math.toRadians(280)).build();
        if (distanceSide < 3 || distanceSide > 6){
            returnSequence = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(.1,() -> {
                        robot.Arm.half();
                    })
                    .addTemporalMarker(2,()->{
                        if (curCycle == 0){
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.LONG_AUTO);
                        }else{
                            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TWO_PLUS_TWO_OUTTAKE);
                        }
                    })
                    .addTemporalMarker(2,() -> {
                        if (curCycle == 0){
                            robot.Arm.setPosition(armState.outtaking);
                            robot.wrist.setPosition(armState.outtaking);
                        }else{
                            robot.Arm.autoOuttake();
                            robot.wrist.autoOuttake();
                        }
                    })
                    .lineToConstantHeading(new Vector2d(4, startPose.getY()))
                    .lineToSplineHeading(new Pose2d(4,-75,Math.toRadians(90)))
                    .splineToConstantHeading(new Vector2d(25,-105),Math.toRadians(280)).build();
        }
        return returnSequence;
    }
    public TrajectorySequence toBoard(Robot robot, SampleMecanumDrive drive, Pose2d startPose,double aprilLoc){
        TrajectorySequence returnSequence = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(aprilLoc,-95.5,Math.toRadians(90))).build();
        return returnSequence;
    }
    public TrajectorySequence goNextToWall(Robot robot, SampleMecanumDrive drive, Pose2d startPose, String TSEloc, String parkType, String pathType){
        robot.Claw.openRight();
        TrajectorySequence returnSequence;
        if (pathType.equals("2+2")){
            returnSequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(.5,() -> {
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TOPSTACK);
                    })
                    .addTemporalMarker(1,() -> {
                        robot.wrist.topStack();
                        robot.Arm.topStack();
                    })
                    .lineToConstantHeading(new Vector2d(1,-62)).build();
        }else {
            returnSequence = park(robot,drive,startPose,parkType);
        }
        return returnSequence;
    }

    public TrajectorySequence backThroughTruss(Robot robot, SampleMecanumDrive drive, Pose2d startPose,Double distanceSide){
        robot.Claw.setPosition(armState.open);
        TrajectorySequence returnSequence = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(8,-15,Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(27,10),Math.toRadians(280)).build();
        if (distanceSide < 6 || distanceSide > 10){
            returnSequence = drive.trajectorySequenceBuilder(startPose)
                    .lineToConstantHeading(new Vector2d(8, startPose.getY()))
                    .lineToSplineHeading(new Pose2d(8,-15,Math.toRadians(90)))
                    .splineToConstantHeading(new Vector2d(27,10),Math.toRadians(280)).build();
        }
        return returnSequence;
    }

    public TrajectorySequence goToStack(Robot robot, SampleMecanumDrive drive, Pose2d startPose, double distanceSide){
        TrajectorySequence returnSequence = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(21, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(26,0))
                .build();
        return returnSequence;
    }
    public TrajectorySequence intakeStackAndReset(Robot robot, SampleMecanumDrive drive, Pose2d startPose) {
        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.ZERO);
        robot.Claw.setPosition(armState.close);
        TrajectorySequence returnSequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(5,-29,Math.toRadians(90)))
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
                .lineToConstantHeading(new Vector2d(20,-84))
                .lineToLinearHeading(new Pose2d(-2,-84,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(-2,-92))
                .build();
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
                    .lineToConstantHeading(new Vector2d(20,-84))
                    .lineToLinearHeading(new Pose2d(50,-84,Math.toRadians(0))).build();
        }
        return returnSequence;
    }
}
