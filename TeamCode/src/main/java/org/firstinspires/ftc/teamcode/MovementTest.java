package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by bense on 12/9/2016.
 */
@TeleOp(name = "Movement Test", group = "Testing")
@Disabled
public class MovementTest extends OpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    GyroSensor gyro;

    int target, startDegrees, targetDegrees, deg;
    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
    }

    public void loop()
    {
        int heading = gyro.getHeading();
        if (heading > 180)
            heading -= 360;

        if (gamepad1.dpad_left)
            deg = 90;
        if (gamepad1.dpad_up)
            deg = 180;
        if (gamepad1.dpad_right)
            deg = 270;
        if (gamepad1.dpad_down)
            deg = 0;

        if (gamepad1.a) {
            navigateBlind(deg, .5, heading);
            telemetry.addData("Status", "Running");
        } else if (gamepad1.x) {
            navigateBlind2(deg, .5, heading);
            telemetry.addData("Status", "Running");
        } else {
                allStop();
                telemetry.addData("Status", "Stopped");
        }

        telemetry.addData("Gamepad", gamepad1.a);
        telemetry.addData("Heading", heading);
    }

    public void navigateBlind(int deg, double power, int h)
    {
        double x = Math.cos(deg * (Math.PI/180.0)), y = Math.sin(deg * (Math.PI/180.0));

        double correction = correct(h);
        frontLeft.setPower((-(-y - x)/2) * power + correction);
        backLeft.setPower(((-y + x)/2) * power + correction);
        frontRight.setPower(((y - x)/2) * power + correction);
        backRight.setPower((-(y + x)/2) * power + correction);
    }

    public void navigateBlind2(int deg, double power, int h)
    {
        double x = Math.cos(deg * (Math.PI/180.0)), y = Math.sin(deg * (Math.PI/180.0));

        double correction = newCorrect(h);
        frontLeft.setPower((-(-y - x)/2) * power + correction);
        backLeft.setPower(((-y + x)/2) * power + correction);
        frontRight.setPower(((y - x)/2) * power + correction);
        backRight.setPower((-(y + x)/2) * power + correction);
    }

    public boolean rotate(char direction, int deg, int h)
    {
        if (direction == 'r' && h - deg < 0)
        {
            frontRight.setPower(-.15);
            frontLeft.setPower(-.15);
            backRight.setPower(-.15);
            backLeft.setPower(-.15);
            return false;
        }
        else if (direction == 'l' && h - deg > 0)
        {
            frontRight.setPower(.15);
            frontLeft.setPower(.15);
            backRight.setPower(.15);
            backLeft.setPower(.15);
            return false;
        }
        return true;
    }

    public void allStop()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public double correct(int h)
    {
        if (h > startDegrees + 5)
            return .08;
        if (h < startDegrees - 5)
            return -.08;
        return 0;
    }

    public double newCorrect(int h)
    {
        return (h * .01)/2;
    }
}
