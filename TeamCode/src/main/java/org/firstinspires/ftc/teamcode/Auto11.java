package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by bense on 11/11/2016.
 */
@Autonomous(name = "Red: Shoot/2 Beacon", group = "Red Autonomous")
@Disabled
public class Auto11 extends OpMode {
    int control = 2, target, startDegrees, targetDegrees, correcting = 0, seconaryDegrees, selectedColor;
    DcMotor sweeper;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor shooter;
    GyroSensor gyro;
    Long time, startTime, segmentTime;
    float mmFTCFieldWidth;
    ColorSensor color;
    ColorSensor line;
    UltrasonicSensor sonar;
    Servo pusher;
    String loopNumber;
    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        shooter = hardwareMap.dcMotor.get("shooter");
        gyro = hardwareMap.gyroSensor.get("gyro");
        sonar = hardwareMap.ultrasonicSensor.get("sonar");
        color = hardwareMap.colorSensor.get("color");
        I2cAddr newAddress = new I2cAddr(0x1f);
        line = hardwareMap.colorSensor.get("line");
        line.setI2cAddress(newAddress);
        pusher = hardwareMap.servo.get("pusher");
        pusher.setPosition(0);
        color.enableLed(false);
        sweeper = hardwareMap.dcMotor.get("sweeper");
        line.enableLed(true);


        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro.calibrate();

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        startTime = System.currentTimeMillis();
        time = startTime;
        segmentTime = startTime;
    }

    public void loop()
    {
        time = System.currentTimeMillis();

        int heading = gyro.getHeading();
        if (heading > 180)
             heading -= 360;

        switch (control)
        {
            case 0: {//Set up for initial delay
                segmentTime = time;
                control = 1;
                telemetry.addData("Status", "Setting up start delay...");
                break;
            }
            case 1: {//Initial delay, set control to 2 to skip delay
                if (segmentTime + 3000 < time) //set to 3 seconds for testing
                    control = 2;
                telemetry.addData("Status", "Waiting to start...");
                break;
            }
            case 2: {//setup for first move
                startDegrees = heading;
                segmentTime = time;
                control = 3;
                telemetry.addData("Status", "Preparing to move...");
                break;
            }
            case 3: {//move into position to shoot (timed move)
                if (navigateTime(180, .5, 1000, heading))
                    control = 4;
                telemetry.addData("Status", "Moving for 1 seconds...");
                break;
            }
            case 4: {//Prepare to shoot
                allStop();
                control = 8;
                break;
            }
            case 8: {//Shoot
                if (!shoot())
                    control = 9;
                telemetry.addData("Status", "Shooting...");
                break;
            }
            case 9: {//move until line is found
                navigateBlind(120, .3, heading);
                if (line.alpha() > 20) {
                    control = 5;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Moving to find beacon...");
                break;
            }
            case 5: {
                allStop();
                if (segmentTime + 500 < time)
                    control = 6;
                break;
            }
            case 6: {
                if (reallign(heading))
                    control = 11;
                break;
            }
            case 11: {
                allStop();
                if (segmentTime + 500 < time) {
                    control = 12;
                    seconaryDegrees = heading;
                }
                break;
            }
            case 12: {//attempt to lineup
                if (line.alpha() > 20) {
                    backRight.setPower(-.2);
                    backLeft.setPower(-.2);
                    frontLeft.setPower(.2);
                    frontRight.setPower(.2);
                    correcting = 0;
                } else if (correcting == 0) {
                    if (heading >= seconaryDegrees)
                        correcting = 1;
                    if (heading < seconaryDegrees)
                        correcting = -1;
                }
                if (correcting != 0) {
                    if (correcting > 0) {
                        backRight.setPower(-.1);
                        backLeft.setPower(-.1);
                        frontLeft.setPower(.2);
                        frontRight.setPower(.2);
                    } else if (correcting < 0) {
                        backRight.setPower(-.2);
                        backLeft.setPower(-.2);
                        frontLeft.setPower(.1);
                        frontRight.setPower(.1);
                    }
                }
                if (sonar.getUltrasonicLevel() <= 20 && sonar.getUltrasonicLevel() > 0) {
                    control = 13;
                    segmentTime = time;
                }

                telemetry.addData("Status", "Lining up...");
                break;
            }
            case 13: {
                if (reallign(heading) && segmentTime + 1000 < time ) {
                    if (line.alpha() < 20)
                        navigateBlind(180, .3, heading);
                    else {
                        allStop();
                        control = 14;
                    }
                }
                break;
            }
            case 14: {//check beacon
                allStop();
                if (color.blue() > color.red()) {
                    pusher.setPosition(1);
                    selectedColor = 1;
                    telemetry.addData("Status", "Blue light detected");
                }
                else {
                    pusher.setPosition(0);
                    selectedColor = 2;
                    telemetry.addData("Status", "Red light detected");
                }
                control = 15;
                segmentTime = time;

                break;
            }
            case 15: {//Give time for servo to move
                allStop();
                if (segmentTime + 1000 < time) {
                    control = 16;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Servo moving...");
                break;
            }
            case 16: {//press a button
                if (segmentTime + 500 > time) {
                    if (selectedColor == 1) {
                        frontLeft.setPower(.3);
                        frontRight.setPower(.3);
                    } else if (selectedColor == 2) {
                        backLeft.setPower(-.3);
                        backRight.setPower(-.3);
                    }
                } else {
                    control = 33;
                    allStop();
                }
                telemetry.addData("Status", "Pressing button...");
                break;
            }
            case 33: {
                if (reallign(heading))
                    control = 17;
                break;
            }
            case 17: {//Move backwards until 30mm away from beacon
                navigateBlind(270, .35, heading);
                if (sonar.getUltrasonicLevel() >= 25) {
                    control = 18;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Moving away from beacon...");
                break;
            }
            case 18: {
                if (segmentTime + 500 < time) {
                    control = 19;
                    segmentTime = time;
                }
                break;
            }
            case 19: {
                if (navigateTime(170, .6, 1000, heading))
                    control = 20;
                break;
            }
            case 20: {
                navigateBlind(170, .3, heading);
                if (line.alpha() > 20) {
                    control = 21;
                    allStop();
                    segmentTime = time;
                }
                break;
            }
            case 21: {
                if (reallign(heading))
                    control = 22;
                break;
            }
            case 22: {
                allStop();
                if (segmentTime + 500 < time) {
                    control = 23;
                    seconaryDegrees = heading;
                }
                break;
            }
            case 23: {//attempt to lineup
                /*if (line.alpha() > 20) {
                    backRight.setPower(-.2);
                    backLeft.setPower(-.2);
                    frontLeft.setPower(.2);
                    frontRight.setPower(.2);
                    correcting = 0;
                } else if (correcting == 0) {
                    if (heading >= seconaryDegrees)
                        correcting = 1;
                    if (heading < seconaryDegrees)
                        correcting = -1;
                }
                if (correcting != 0) {
                    if (correcting > 0) {
                        backRight.setPower(-.1);
                        backLeft.setPower(-.1);
                        frontLeft.setPower(.2);
                        frontRight.setPower(.2);
                    } else if (correcting < 0) {
                        backRight.setPower(-.2);
                        backLeft.setPower(-.2);
                        frontLeft.setPower(.1);
                        frontRight.setPower(.1);
                    }
                }*/

                navigateBlind(90, .3, heading);

                if (sonar.getUltrasonicLevel() <= 20 && sonar.getUltrasonicLevel() > 0) {
                    control = 31;
                    allStop();
                    segmentTime = time;
                }

                telemetry.addData("Status", "Lining up...");
                break;
            }
            case 31: {
                if (reallign(heading))
                    control = 32;
                break;
            }
            case 32: {
                navigateBlind(270, .3, heading);
                if (sonar.getUltrasonicLevel() > 15) {
                    control = 24;
                    allStop();
                }
                break;
            }
            case 24: {
                if (reallign(heading) && segmentTime + 1000 < time ) {
                    if (line.alpha() < 20)
                        navigateBlind(0, .3, heading);
                    else {
                        allStop();
                        control = 25;
                    }
                }
                break;
            }
            case 25: {//check beacon
                allStop();
                if (color.blue() > color.red()) {
                    pusher.setPosition(1);
                    selectedColor = 1;
                    telemetry.addData("Status", "Blue light detected");
                }
                else {
                    pusher.setPosition(0);
                    selectedColor = 2;
                    telemetry.addData("Status", "Red light detected");
                }
                control = 26;
                segmentTime = time;

                break;
            }
            case 26: {//Give time for servo to move
                allStop();
                if (segmentTime + 1000 < time) {
                    control = 27;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Servo moving...");
                break;
            }
            case 27: {//press a button
                if (segmentTime + 500 > time) {
                    if (selectedColor == 1) {
                        frontLeft.setPower(.3);
                        frontRight.setPower(.3);
                    } else if (selectedColor == 2) {
                        backLeft.setPower(-.3);
                        backRight.setPower(-.3);
                    }
                } else {
                    control = 34;
                    allStop();
                }
                telemetry.addData("Status", "Pressing button...");
                break;
            }
            case 34: {
                if (reallign(heading))
                    control = 28;
                break;
            }
            case 28: {//Move backwards until 30mm away from beacon
                navigateBlind(270, .35, heading);
                if (sonar.getUltrasonicLevel() >= 30) {
                    control = 99;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Moving away from beacon...");
                break;
            }


            default: {//Hopefully this only runs when program ends
                allStop();
                telemetry.addData("Status", "Switch is in default. Waiting for autonomous to end...");
            }
        }

        telemetry.addData("Line", line.alpha());
        telemetry.addData("Timer", time - segmentTime);
        telemetry.addData("Control", control);
        telemetry.addData("Heading", heading);
        telemetry.addData("Sonar", sonar.getUltrasonicLevel());
        telemetry.addData("Target degrees", targetDegrees);
        if (color.blue() > color.red())
            telemetry.addData("Color", "Blue");
        else if (color.red() > color.blue())
            telemetry.addData("Color", "Red");
        else
            telemetry.addData("Color", "Unknown");
    }

    public boolean navigateTime(int deg, double power, long targetTime, int h) //like unit circle, 90 forwards, 270 backwards
    {
        double x = Math.cos(deg * (Math.PI/180.0)), y = Math.sin(deg * (Math.PI/180.0));

        if (segmentTime + targetTime > time)
        {
            double correction = correct(h);
            frontLeft.setPower((-(-y - x)/2) * power + correction);
            backLeft.setPower(((-y + x)/2) * power + correction);
            frontRight.setPower(((y - x)/2) * power + correction);
            backRight.setPower((-(y + x)/2) * power + correction);

            return false;
        }
        return true;
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

    public boolean rotate(char direction, int deg, int h)
    {
        target = deg;
        if (direction == 'r' && h - target < 0)
        {
            frontRight.setPower(-.15);
            frontLeft.setPower(-.15);
            backRight.setPower(-.15);
            backLeft.setPower(-.15);
            return false;
        }
        else if (direction == 'l' && h - target > 0)
        {
            frontRight.setPower(.15);
            frontLeft.setPower(.15);
            backRight.setPower(.15);
            backLeft.setPower(.15);
            return false;
        }
        return true;
    }

    public void allStop()//Stops all drive motors
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public double correct(int h)
    {
        if (h > startDegrees + 3)
            return .05;
        if (h < startDegrees - 3)
            return -.05;
        return 0;
    }

    public boolean shoot() //Waiting for launcher to be built, no code implemented
    {
        boolean returnstatement = false;

        if (shooter.getCurrentPosition() > -720){
            shooter.setPower(1);
            loopNumber = "If statement";
            returnstatement = true;
        }

        else if (shooter.getCurrentPosition() <= -720 && shooter.getCurrentPosition() > shooter.getTargetPosition()) {
            shooter.setPower(.5);
            loopNumber = "Else if statement 1";
            returnstatement = true;
        }
        else if (shooter.getCurrentPosition() <= shooter.getTargetPosition()) {
            shooter.setPower(0);
            loopNumber = "Else if statement 2";
            returnstatement = false;
        }
        return returnstatement;
    }

    public boolean reallign (int h)
    {
        if (h + 3 < 0) {
            frontRight.setPower(-.08);
            frontLeft.setPower(-.08);
            backRight.setPower(-.08);
            backLeft.setPower(-.08);
            segmentTime = time;
        } else if (h - 3 > 0) {
            frontRight.setPower(.08);
            frontLeft.setPower(.08);
            backRight.setPower(.08);
            backLeft.setPower(.08);
            segmentTime = time;
        } else {
            allStop();
            if (segmentTime + 250 < time)
                return true;
        }
        return false;
    }
}
