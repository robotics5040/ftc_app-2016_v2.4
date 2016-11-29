package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by bense on 11/9/2016.
 */
@Autonomous(name = "ColorTest2", group = "Testing")
@Disabled
public class ColorTest2 extends OpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    ColorSensor color, line, line2;
    GyroSensor gyro;
    Servo pusher;
    int heading, degrees = 0, previousHeading = 0, trueHeading, startDegrees = 0;

    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        color = hardwareMap.colorSensor.get("color");
        I2cAddr colorAddress = new I2cAddr(0x1d);
        color.setI2cAddress(colorAddress);
        line = hardwareMap.colorSensor.get("line");
        gyro = hardwareMap.gyroSensor.get("gyro");
        pusher = hardwareMap.servo.get("pusher");
        pusher.setPosition(.5);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop()
    {
        heading = gyro.getHeading();
        checkHeading();
        trueHeading = degrees + heading;

        if (line.blue() >= 5)
            pusher.setPosition(1);
        else
            pusher.setPosition(0);

        previousHeading = gyro.getHeading();
    }

    public void navigateBlind(int deg, double power)
    {
        double x = Math.cos(deg * (Math.PI/180.0)), y = Math.sin(deg * (Math.PI/180.0));

        double correction = correct();
        frontLeft.setPower((-(-y - x)/2) * power + correction);
        backLeft.setPower(((-y + x)/2) * power + correction);
        frontRight.setPower(((y - x)/2) * power + correction);
        backRight.setPower((-(y + x)/2) * power + correction);
    }

    public double correct()
    {
        if (trueHeading > startDegrees + 3)
            return .1;
        if (trueHeading < startDegrees - 3)
            return -.1;
        return 0;
    }

    public void allStop()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void checkHeading()
    {
        if (previousHeading - heading > 270)
            degrees += 360;
        else if (heading - previousHeading > 270)
            degrees -= 360;
    }
}
