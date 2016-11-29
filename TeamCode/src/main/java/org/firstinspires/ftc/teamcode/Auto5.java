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
@Autonomous(name = "Red3", group = "Red Autonomous")
@Disabled
public class Auto5 extends OpMode {
    int control = 2, target, startDegrees, targetDegrees;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    GyroSensor gyro;
    float robotBearing;
    Long time, startTime, segmentTime;
    VuforiaLocalizer vuforia;
    List<VuforiaTrackable> allTrackables;
    double posx, posy, startx, starty, targetDistance;
    float mmFTCFieldWidth;
    ColorSensor color;
    ColorSensor line;
    UltrasonicSensor sonar;
    Servo pusher;
    boolean lineUsed = false, targetActive;

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;
    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        gyro = hardwareMap.gyroSensor.get("gyro");
        sonar = hardwareMap.ultrasonicSensor.get("sonar");
        color = hardwareMap.colorSensor.get("color");
        I2cAddr newAddress = new I2cAddr(0x1f);
        line = hardwareMap.colorSensor.get("line");
        line.setI2cAddress(newAddress);
        pusher = hardwareMap.servo.get("pusher");
        pusher.setPosition(0);
        color.enableLed(false);
        line.enableLed(true);

        gyro.calibrate();

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        startTime = System.currentTimeMillis();
        time = startTime;
        segmentTime = startTime;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AUBrQCz/////AAAAGXg5njs2FEpBgEGX/o6QppZq8c+tG+wbAB+cjpPcC5bwtGmv+kD1lqGbNrlHctdvrdmTJ9Fm1OseZYM15VBaiF++ICnjCSY/IHPhjGW9TXDMAOv/Pdz/T5H86PduPVVKvdGiQ/gpE8v6HePezWRRWG6CTA21itPZfj0xDuHdqrAGGiIQXcUbCTfRAkY7HwwRfQOM1aDhmeAaOvkPPCnaA228iposAByBHmA2rkx4/SmTtN82rtOoRn3/I1PA9RxMiWHWlU67yMQW4ExpTe2eRtq7fPGCCjFeXqOl57au/rZySASURemt7pwbprumwoyqYLgK9eJ6hC2UqkJO5GFzTi3XiDNOYcaFOkP71P5NE/BB";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables targets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        VuforiaTrackable redTools  = targets.get(1); //load tools
        redTools.setName("Tools");

        VuforiaTrackable redGears  = targets.get(3); //load gears
        redGears.setName("Gears");

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = (float)16.5 * mmPerInch;
        mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;

        OpenGLMatrix redToolsLocationOnField = OpenGLMatrix //set up tracking for tools
                .translation(mmFTCFieldWidth/2 - (float)863.6, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        redTools.setLocation(redToolsLocationOnField);
        RobotLog.ii(TAG, "Tools=%s", format(redToolsLocationOnField));

        OpenGLMatrix redGearsLocationOnField = OpenGLMatrix //set up tracking for gears
                .translation(mmFTCFieldWidth/2 - (float)2082.8, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        redGears.setLocation(redGearsLocationOnField);
        RobotLog.ii(TAG, "Gears=%s", format(redGearsLocationOnField));

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix //set up phone
                .translation(mmBotWidth/2,(float)44.45 + 175,200)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, -90, -90, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)redTools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)redGears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        targets.activate();
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
            case 4: {//Setup for slight correction
                allStop();
                control = 8;
                break;
            }
            case 8: {//setup for move
                control = 9;
                telemetry.addData("Status", "Preparing to move...");
                break;
            }
            case 9: {//move until target is visible
                navigateBlind(120, .3, heading);
                targetActive = scan(allTrackables.get(3));
                if (line.alpha() > 20) {
                    control = 5;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Moving to find beacon...");
                break;
            }
            case 5: {
                scan(allTrackables.get(3));
                allStop();
                if (segmentTime + 500 < time)
                    control = 6;
                break;
            }
            case 6: {
                scan(allTrackables.get(3));
                if (heading + 2 < 0) {
                    frontRight.setPower(-.1);
                    frontLeft.setPower(-.1);
                    backRight.setPower(-.1);
                    backLeft.setPower(-.1);
                } else if (heading - 2 > 0) {
                    frontRight.setPower(.1);
                    frontLeft.setPower(.1);
                    backRight.setPower(.1);
                    backLeft.setPower(.1);
                } else {
                    allStop();
                    control = 7;
                }
                break;
            }
            case 7: {
                allStop();
                scan(allTrackables.get(3));
                double targetx = posx - -510;
                double targety = posy - 1475;

                if (targetx < 0)
                    targetDegrees = 180 - (int) ((180 / Math.PI) * (Math.atan(targety / targetx)));
                else if (targetx > 0)
                    targetDegrees = (int) -((180 / Math.PI) * (Math.atan(targety / targetx)));
                else
                    targetDegrees = 90;

                //targetDegrees -= heading;
                control = 12;
                break;
            }
            case 10: {
                allStop();
                targetActive = scan(allTrackables.get(3));
                if (segmentTime + 1000 < time)
                    control = 11;
                break;
            }
            case 11: {//setup for lineup
                targetActive = scan(allTrackables.get(3));
                System.out.println("AUTO1 LOG: LOG START");

                //Target x: -510     Target y: 1475
                double targetx = posx - -510;
                double targety = posy - 1200;
                startx = posx;
                starty = posy;

                if (targetx > 0)
                    targetDegrees = (int) ((180 / Math.PI) * (Math.atan(targety / targetx)));
                else if (targetx < 0)
                    targetDegrees = 180 + (int) ((180 / Math.PI) * (Math.atan(targety / targetx)));
                else
                    targetDegrees = 90;
                System.out.println("AUTO1 LOG: startx: " + startx + "   starty: " + starty + "   targetx: " + targetx + "   targety: " + targety + "   targetDegrees: " + targetDegrees);
                //targetDistance = Math.sqrt(Math.pow(Math.abs(targetx), 2) + Math.pow(Math.abs(targety), 2));
                control = 12;
                telemetry.addData("target degrees", targetDegrees);

                telemetry.addData("Status", "Calculating initial angle...");
                break;
            }
            case 12: {//attempt to lineup
                if (line.red() < 10) {
                    navigateBlind(targetDegrees, .3, heading);
                    lineUsed = false;
                } else {
                    navigateBlind(90, .3, heading);
                    lineUsed = true;
                }
                if (sonar.getUltrasonicLevel() <= 10 && sonar.getUltrasonicLevel() > -1)
                    control = 99;

                //System.out.println("AUTO1 LOG: targetx: " + targetx + "   targety: " + targety + "   targetDegrees: " + targetDegrees + "   lineUsed: " + lineUsed);

                telemetry.addData("Status", "Lining up...");
                telemetry.addData("posx", posx);
                telemetry.addData("posy", posy);
                //telemetry.addData("targetx", targetx);
                //telemetry.addData("targety", targety);
                break;
            }
            case 13: {//check lineup, adjust if veering away from target
                targetActive = scan(allTrackables.get(3));
                double targetx = posx - -515;
                double targety = posy - 1200;

                if (targetx < 0)
                    targetDegrees = (int) ((180 / Math.PI) * (Math.atan(targety / targetx)));
                else if (targetx > 0)
                    targetDegrees = 180 + (int) ((180 / Math.PI) * (Math.atan(targety / targetx)));
                else
                    targetDegrees = 90;
                System.out.println("AUTO1 LOG: targetx: " + targetx + "   targety: " + targety + "   targetDegrees: " + targetDegrees + "   lineUsed: " + lineUsed);
                telemetry.addData("Status", "Checking lineup...");
                control = 12;
                segmentTime = time;
                break;
            }
            case 14: {//check beacon
                System.out.println("AUTO1 LOG: SEGMENT 1 END");
                allStop();
                if (color.blue() > color.red()) {
                    pusher.setPosition(1);
                    telemetry.addData("Status", "Blue light detected");
                }
                else {
                    pusher.setPosition(0);
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
                if (navigateTime(90, .5, 500, heading))
                    control = 17;
                telemetry.addData("Status", "Pressing button...");
                break;
            }
            case 17: {//Move backwards until 50mm away from beacon
                navigateBlind(270, .35, heading);
                if (sonar.getUltrasonicLevel() >= 50) {
                    control = 19;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Moving away from beacon...");
                break;
            }
            case 18: {
                boolean lineup = true;
                targetActive = scan(allTrackables.get(3));
                if (heading + 3 < startDegrees && heading - 3 > startDegrees) {
                    frontRight.setPower(correct(heading) * 1.5);
                    frontLeft.setPower(correct(heading) * 1.5);
                    backRight.setPower(correct(heading) * 1.5);
                    backLeft.setPower(correct(heading) * 1.5);
                    lineup = false;
                }
                if (line.blue() < 5) {
                    if (posx < -510)
                        navigateBlind(180, .15, heading);
                    if (posx > -510)
                        navigateBlind(0, .15, heading);
                    lineup = false;
                }
                if (lineup)
                    control = 14;

            }
            case 19: {
                allStop();
                control = 20;
                break;
            }
            case 20: {
                if (rotate('r', -90, heading)) {
                    control = 21;
                    segmentTime = System.currentTimeMillis();
                }
                break;
            }
            case 21: {
                if (segmentTime + 500 < time)
                    control = 22;
                break;
            }
            case 22: {
                shoot();
                control = 23;
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

        if (lastLocation != null) {
            VectorF trans = lastLocation.getTranslation();
            Orientation rot = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
            posx = trans.get(0);
            posy = trans.get(1);

            robotBearing = rot.thirdAngle;

            telemetry.addData("posx", posx);
            telemetry.addData("posy", posy);

            telemetry.addData("Pos", format(lastLocation));
        } else {
            telemetry.addData("Pos", "Unknown");
        }
    }

    public boolean navigate(int deg, double power, double distance, int h) //like unit circle, 90 forwards, 270 backwards
    {
        double x = Math.cos(deg * (Math.PI/180.0)), y = Math.sin(deg * (Math.PI/180.0));
        double targetx = distance * x + startx;
        double targety = distance * y + starty;

        telemetry.addData("target x", targetx);
        telemetry.addData("target y", targety);

        if (targetx + startx <= posx || targety + starty >= posy)
        {
            double correction = correct(h); //Course correction
            frontLeft.setPower((-(-y - x)/2) * power + correction);
            backLeft.setPower(((-y + x)/2) * power + correction);
            frontRight.setPower(((y - x)/2) * power + correction);
            backRight.setPower((-(y + x)/2) * power + correction);

            return false;
        }
        return true;
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

    /*public void checkHeading()
    {
        if (previousHeading - heading > 270)
            degrees += 360;
        else if (heading - previousHeading > 270)
            degrees -= 360;
    }*/

    public void allStop()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        //will add all motors when they are added
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public double correct(int h)
    {
        if (h > startDegrees + 10)
            return .05;
        if (h < startDegrees - 10)
            return -.05;
        return 0;
    }

    public boolean shoot() //Waiting for launcher to be built, no code implemented
    {
        return true;
    }

    public boolean scan(VuforiaTrackable t) //for t, use allTrackables.get(). 0 is Wheels, 1 is Tools, 2 is Legos, 3 is Gears
    {
        telemetry.addData(t.getName(), ((VuforiaTrackableDefaultListener) t.getListener()).isVisible() ? "Visible" : "Not Visible");    //

        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) t.getListener()).getUpdatedRobotLocation();
        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;
        }

        if (((VuforiaTrackableDefaultListener) t.getListener()).isVisible())
            return true;
        return false;
    }
}
