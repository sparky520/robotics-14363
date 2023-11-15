package org.firstinspires.ftc.teamcode.SubSystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Servo;
public class Arm {
    private GamepadEx driver, operator;
    private Servo armLeft;
    private Servo armRight;
    int armTarget = 0;
    String armCurrentDirection = "down";
    public Arm(){

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

    }
    public void up()
    {
        armTarget = 200;
        armCurrentDirection = "up";

        armLeft.setPosition(armTarget);
        armRight.setPosition(armTarget);

    }
    public void down()
    {
        armTarget = 0;
        armCurrentDirection = "down";


        armLeft.setPosition(armTarget);
        armRight.setPosition(armTarget);

    }
}

