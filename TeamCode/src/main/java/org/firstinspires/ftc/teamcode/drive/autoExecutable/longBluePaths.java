package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class longBluePaths {

    longBluePaths(){

    }
    public TrajectorySequence goNextToWall(Robot robot, SampleMecanumDrive drive, Pose2d startPose, String TSEloc){
        TrajectorySequence returnSequence;
        if (TSEloc.equals("RIGHT")){
            returnSequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(.5,() -> {
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TOPSTACK);
                    })
                    .addTemporalMarker(1,() -> {
                        robot.wrist.topStack();
                        robot.Arm.topStack();
                    })
                    .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY() - 6))
                    .lineToConstantHeading(new Vector2d(1,62)).build();
            return returnSequence;
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
            return returnSequence;
        }
    }
}
