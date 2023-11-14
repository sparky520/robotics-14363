package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    private GamepadEx driver, operator;
    private Servo armLeft;
    private Servo armRight;
    int armTarget = 0;
    double armSpeed = 0;
    String armCurrentDirection = "down";
    public Arm(HardwareMap hardwareMap){

        armLeft = hardwareMap.servo.get("armLeft");
        armRight = hardwareMap.servo.get("armRight");

        // Reverse backwards arm motor
        armRight.setDirection(Servo.Direction.REVERSE);

        //get starting position
        armLeft.getPosition();
        armRight.getPosition();

        // Set arm encoders to 0
        armLeft.setPosition(0);
        armRight.setPosition(0);


        public void upanddown()
        {
            if (driver.wasJustPressed(GamepadKeys.Button.A)){ // Arm UP
                armTarget = 200;
                armSpeed = 0.98;
                armCurrentDirection = "up";

                armLeft.setPower(armSpeed);
                armLeft.setPosition(armTarget);

            } else if (driver.wasJustPressed(GamepadKeys.Button.B)) { // Arm DOWN
                armTarget = 0;
                armSpeed = -0.6;
                armCurrentDirection = "down";

                armLeft.setPower(armSpeed);
                armLeft.setPosition(armTarget);
            }
        }
    }
}

