package org.firstinspires.ftc.teamcode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class LeftBluePark extends LinearOpMode{
    //current position
    Pose2d myPose = new Pose2d(-63, 12, Math.toRadians(-180));
    public void runOpMode(){
        SampleMecanumDrive park = new SampleMecanumDrive(hardwareMap);

        park.setPoseEstimate(myPose);

        Trajectory trajectory = park.trajectoryBuilder(myPose).strafeRight(50).build();

        if (isStopRequested()) return;

        park.followTrajectory(trajectory);

    }
}
