package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous (name="caveman type parking")
public class blueAuto extends LinearOpMode {

    private DcMotorEx rightFront, leftFront, rightRear, leftRear;

    double mult = 1.0;
    double batteryVoltage;
    String voltageCategory;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while (opModeIsActive()) {

            forward(1.0, 5);
            strafeLeft(.5, 3);
            backwards(1.0, 5);
            strafeLeft(.5, 3);
            stopMotors();
            break;


        }

    }

    private void forward(double mult, double sec) {
        double timer = (getRuntime() + sec);

        while (time > getRuntime()) {
            rightFront.setPower(mult);
            leftFront.setPower(mult);
            rightRear.setPower(mult);
            leftRear.setPower(mult);

        }
    }

    private void backwards(double mult, double sec) {
        double timer = (getRuntime() + sec);

        while (timer > getRuntime()) {
            rightFront.setPower(mult * -1);
            leftFront.setPower(mult * -1);
            rightRear.setPower(mult * -1);
            leftRear.setPower(mult * -1);
        }
    }

    private void strafeLeft(double mult, double sec) {
        rightFront.setPower(mult * 1);
        leftFront.setPower(mult * -1);
        rightRear.setPower(mult * -1);
        leftRear.setPower(mult * 1);
        runFor(sec);
    }

    // drive back

    // drive right
    private void strafeRight(double mult, double sec) {
        rightFront.setPower(mult * -1);
        leftFront.setPower(mult * 1);
        rightRear.setPower(mult * 1);
        leftRear.setPower(mult * -1);
        runFor(sec);
    }

    private void stopMotors() {
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
    }

    private void runFor(double sec) {
        //sleeps for given time, so the program can run. FTC sleep means keep doing what you are doing, not stop everything
        sleep((long) (1000 * sec));

    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void voltageTelem() {

        batteryVoltage = getBatteryVoltage();
        boolean voltNotFound = false;

        if (batteryVoltage >= 14) {
            voltageCategory = "Above 14V";

        } else if (batteryVoltage >= 13) {
            voltageCategory = "13-14V";

        } else if (batteryVoltage >= 12) {
            voltageCategory = "12-13V";

        } else if (batteryVoltage >= 11) {
            voltageCategory = "11-12V";

        } else {

            voltageCategory = "Below 11V";
        }
        switch (voltageCategory){
            case "Above 14V":
                mult = 0.8;
                break;
            case "13-14V":
                mult = 0.85;
                break;
            case "12-13V":
                mult = 0.9;
                break;
            case "11-12V":
                mult = 1;
                break;
            default:
                telemetry.addLine("Voltage too weak");
                voltNotFound = true;
            {
                if(!voltNotFound)
                {
                    telemetry.addData("Current Battery voltage " , batteryVoltage);
                    telemetry.addData("Current speed multiplier: " , mult);
                }
                telemetry.update();
            }
        }
    }

}



