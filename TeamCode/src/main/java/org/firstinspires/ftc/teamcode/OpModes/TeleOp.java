package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private GamepadEx driver, operator;
    private Robot robot;
    public TouchSensor touchSensor;
    public ElapsedTime timer = new ElapsedTime();
    ColorRangeSensor claw1,claw2,color1;
    double sensor1Val, sensor2Val, sensor3Val;
    enum state{
        outtaking,intaking
    }
    boolean outtaking = false;
    DistanceSensor distanceSensor3,distanceSensor2, distanceSensor;
    boolean autoOuttake = false;
    double backDistance, sideDistance;
    boolean switchedOuttakeTypeThisLoop = false;
    outtakeStates currentSlideState = outtakeStates.STATION;
    double distanceFront, distanceSide, distanceBack;
    state currentState = state.intaking;
    @Override
    public void init()
    {
        telemetry.setMsTransmissionInterval(50);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, telemetry);
        robot.wrist.setPosition(armState.intakingCLAW);
        robot.Arm.setPosition(armState.low);
        robot.Claw.setPosition(armState.open);
        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.STATION);
        color1 = hardwareMap.get(ColorRangeSensor.class, "colorBoard");
        claw1 = hardwareMap.get(ColorRangeSensor.class, "claw1");
        claw2 = hardwareMap.get(ColorRangeSensor.class, "claw2");

    }
    @Override
    public void loop() {
        driver.readButtons();
        operator.readButtons();

        telemetry.addData("LSlide pos", robot.slidev2.leftSlide.getCurrentPosition());
        telemetry.addData("RSlide pos", robot.slidev2.rightSlide.getCurrentPosition());
        telemetry.addData("going up", robot.slidev2.goingUp);
        telemetry.addData("state", robot.slidev2.currentSlideState);

        robot.drivetrain.fieldCentric(driver);
        switch ()
        sensor1Val = claw1.blue()+claw1.red()+claw1.green();
        sensor2Val = claw2.blue()+claw2.red()+claw2.green();
        sensor3Val = color1.blue()+color1.red()+color1.green();


        if (gamepad2.right_trigger > 0 && sensor3Val > 200 && outtaking && backDistance < 5){
            robot.Claw.setPosition(armState.open);
        }
        if (sensor1Val > 175 && !outtaking){
            robot.Claw.closeRight();
        }
        if (sensor2Val > 175 && !outtaking){
            robot.Claw.closeLeft();
        }


        if (gamepad2.circle){
            outtaking = true;
            robot.Arm.setPosition(armState.outtaking);
            robot.wrist.setPosition(armState.outtaking);
        }
        if (gamepad2.triangle){
            outtaking = false;
            robot.Arm.setPosition(armState.medium);
        }
        if (gamepad2.square){
            outtaking = false;
            robot.Arm.setPosition(armState.low);
            robot.wrist.setPosition(armState.intakingCLAW);
            robot.Claw.setPosition(armState.open);
        }
        if (gamepad2.dpad_up){

            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.MEDIUM);
            //robot.slidev2.isGoingUp(1100);
            //robot.slidev2.currentSlideState = outtakeStates.MEDIUMIN;
        }
        if (gamepad2.dpad_left){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.LOW);
        }
        if (gamepad2.dpad_right){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.HIGH);
            outtaking = true;
            robot.Arm.highOuttake();
            robot.wrist.highOuttake();
        }

        if (gamepad2.dpad_down){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.STATION);
            //robot.slidev2.isGoingUp(100);
            //robot.slidev2.currentSlideState = outtakeStates.LOWIN;
        }
        if (gamepad2.left_bumper){
            robot.slide.driftOffset += 10;
        }
        if (gamepad1.square){
            robot.Airplane.setPosition(armState.airplaneLaunch);
        }
        if (gamepad1.triangle){
            robot.drivetrain.resetIMU();
        }
        if (gamepad1.right_trigger > 0){
            robot.drivetrain.slow_mode = .25;
        }else if (gamepad1.left_trigger > 0){
            robot.drivetrain.slow_mode = .5;
        }
        else{
            robot.drivetrain.slow_mode = 1;
        }
        if (gamepad1.left_bumper){
            robot.Claw.openLeft();
        }
        if (gamepad1.right_bumper){
            robot.Claw.openRight();
        }

        telemetry.update();
    }
    public void initDistance(){
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        distanceSensor3 = hardwareMap.get(DistanceSensor.class, "distanceSensor3");
    }
    public void distanceTelem(){
        distanceFront = distanceSensor.getDistance(DistanceUnit.INCH);
        distanceSide = distanceSensor2.getDistance(DistanceUnit.INCH);
        distanceBack = distanceSensor3.getDistance(DistanceUnit.INCH);
        telemetry.addData("side distance", distanceSide);
        telemetry.addData("front distance", distanceFront);
        telemetry.addData("back distance", distanceBack);
    }
}