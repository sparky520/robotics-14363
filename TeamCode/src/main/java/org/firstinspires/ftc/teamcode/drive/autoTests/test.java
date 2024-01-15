package org.firstinspires.ftc.teamcode.drive.autoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class test extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum state{
        toCenter,
        wait1,
        wait2,
        toBoard,
        wait3,
        moveAwayFromBoard,
        retractArm1,
        toTape,
        toTape2,
        wait4,
        IDLE,
    }

    // We define the current state we're on
    // Default to IDLE
    Robot robot;
    state currentState = state.IDLE;
    ElapsedTime timer = new ElapsedTime();
    DistanceSensor distanceSensor;
    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d start = new Pose2d(0, 0, Math.toRadians(0));
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);
        robot = new Robot(hardwareMap, telemetry);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distancesensor");

        Trajectory center = drive.trajectoryBuilder(start).lineToLinearHeading(new Pose2d(31.5,-6,Math.toRadians(90))).build();
        Trajectory board = drive.trajectoryBuilder(center.end()).lineToConstantHeading(new Vector2d(19.5,-25)).build();
        Trajectory moveAwayFromBoard = drive.trajectoryBuilder(board.end())
                .lineToConstantHeading(new Vector2d(19.5,-23))
                .build();
        Trajectory toTape = drive.trajectoryBuilder(moveAwayFromBoard.end()).lineToConstantHeading(new Vector2d(19.5,15)).build();
        Trajectory toTape2 = drive.trajectoryBuilder(moveAwayFromBoard.end()).lineToConstantHeading(new Vector2d(18.7,19)).build();
        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = state.toCenter;
        robot.Claw.setPosition(armState.intakingCLAW);
        drive.followTrajectoryAsync(center);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch(currentState){
                case toCenter:
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
                        drive.followTrajectory(toBoard(board));
                        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
                        telemetry.update();
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
                        drive.followTrajectoryAsync(moveAwayFromBoard);
                        currentState = state.retractArm1;
                        timer.reset();
                    }
                    break;
                case retractArm1:
                    if (timer.seconds() > .5){
                        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
                        robot.Arm.setPosition(armState.medium);
                        robot.Claw.setPosition(armState.intakingCLAW);
                        currentState = state.toTape;
                    }
                    break;
                case toTape:
                    if (!drive.isBusy()){
                        drive.followTrajectoryAsync(toTape);
                        robot.Arm.topStack();
                        currentState = state.toTape2;
                    }
                case toTape2:
                    if (!drive.isBusy()){
                        drive.followTrajectoryAsync(toTape2);
                        currentState = state.wait4;
                    }
                case wait4:
                    if (!drive.isBusy()){
                        robot.Claw.setPosition(armState.intakingCLAW);
                        currentState = state.IDLE;
                    }
                case IDLE:
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state

            Pose2d poseEstimate = drive.getPoseEstimate();


            // Print pose to telemetry

        }
    }
    public void outtakeAfterMedium(){
        ElapsedTime timer1 = new ElapsedTime();
        while (timer1.seconds() < .6){
            continue;
        }
        robot.Arm.setPosition(armState.high);
        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
    }
    public Trajectory toBoard(Trajectory end){
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        //Trajectory x = drive.trajectoryBuilder(end.end()).forward(distance-12).build();
        double y = distance - 3.5;
        Trajectory x = drive.trajectoryBuilder(end.end()).forward(-y).build();
        return x;
    }
    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop

}