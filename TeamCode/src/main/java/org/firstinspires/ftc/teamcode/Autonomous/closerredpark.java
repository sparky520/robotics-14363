package org.firstinspires.ftc.teamcode.Autonomous;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class closerredpark {
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;
    HardwareMap hardwareMap;
    double mult = 1.0;
    double batteryVoltage;
    String voltageCategory;
    Telemetry telemetry;
    public void init() {

        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRight = hardwareMap.get(DcMotorEx.class, "rightRear");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void init_loop() {

    }

    public void start() {
        forward(.5,2);
        strafeRight(4,1);
    }

    public void loop() {


    }


    private void forward(double mult, double sec){
        frontRight.setPower(mult);
        frontLeft.setPower(mult); //mult changes the speed the motors go. Slower is more consistent
        backRight.setPower(mult);
        backLeft.setPower(mult);
        runFor(sec); //runs for this amount of time
        /* Mecanum forward (+ means forward, - means backwards)
         + +
         + +
        */
    }

    private void backwards(double mult, double sec) {
        frontRight.setPower(mult * -1);
        frontLeft.setPower(mult * -1);
        backRight.setPower(mult * -1);
        backLeft.setPower(mult * -1);
        runFor(sec);
    }
    // drive left
    private void strafeLeft(double mult, double sec) {
        frontRight.setPower(mult * 1);
        frontLeft.setPower(mult * -1);
        backRight.setPower(mult * -1);
        backLeft.setPower(mult * 1);
        runFor(sec);
    }

    // drive back

    // drive right
    private void strafeRight(double mult, double sec) {
        frontRight.setPower(mult * -1);
        frontLeft.setPower(mult * 1);
        backRight.setPower(mult * 1);
        backLeft.setPower(mult * -1);
        runFor(sec);
    }

    private void runFor(double sec){
        //sleeps for given time, so the program can run. FTC sleep means keep doing what you are doing, not stop everything
        sleep((long) (1000 * sec));

    }
}

