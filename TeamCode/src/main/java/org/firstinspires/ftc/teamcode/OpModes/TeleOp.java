package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.SubSystems.*;
import org.firstinspires.ftc.teamcode.SubSystems.arm;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private GamepadEx driver, operator;
    private Robot robot;
    private Mecanum driveTrain;

    private Servo leftArm, rightArm;
    Mecanum wheels;
    @Override
    public void init()
    {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, telemetry);


    }
    @Override
    public void loop() {

        driver.readButtons();
        operator.readButtons();

        robot.drivetrain.fieldCentric(driver);

       if (gamepad1.dpad_up){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
       }
      if (gamepad1.dpad_left){
           robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.MEDIUMIN);
      }
      if (gamepad1.dpad_right){
           robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
      }

      if (gamepad1.left_bumper){
            robot.Arm.setPosition(armState.outtaking);
      }
      if (gamepad1.right_bumper){
            robot.Arm.setPosition(armState.intaking);
      }
      if (gamepad1.circle){
          robot.Claw.setPosition(armState.intaking);
      }
      if (gamepad1.square){
          robot.Claw.setPosition(armState.outtaking);
      }







    }
}