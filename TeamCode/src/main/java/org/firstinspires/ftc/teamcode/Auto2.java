package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
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
 * Created by bense on 10/11/2016.
 */
@Autonomous (name = "Blue 1", group = "Blue Autonomous")
@Disabled
public class Auto2 extends OpMode {
    int control = 2, degrees = 0, previousHeading = 0, heading, trueHeading, target, startDegrees, targetDegrees, initialDegrees = 0;
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
    UltrasonicSensor sonar;
    Servo pusher;

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;
    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        sonar = hardwareMap.ultrasonicSensor.get("sonar");
        color = hardwareMap.colorSensor.get("color");
        pusher = hardwareMap.servo.get("pusher");

        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        pusher.setPosition(0);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        startTime = System.currentTimeMillis();
        time = startTime;
        segmentTime = startTime;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AUBrQCz/////AAAAGXg5njs2FEpBgEGX/o6QppZq8c+tG+wbAB+cjpPcC5bwtGmv+kD1lqGbNrlHctdvrdmTJ9Fm1OseZYM15VBaiF++ICnjCSY/IHPhjGW9TXDMAOv/Pdz/T5H86PduPVVKvdGiQ/gpE8v6HePezWRRWG6CTA21itPZfj0xDuHdqrAGGiIQXcUbCTfRAkY7HwwRfQOM1aDhmeAaOvkPPCnaA228iposAByBHmA2rkx4/SmTtN82rtOoRn3/I1PA9RxMiWHWlU67yMQW4ExpTe2eRtq7fPGCCjFeXqOl57au/rZySASURemt7pwbprumwoyqYLgK9eJ6hC2UqkJO5GFzTi3XiDNOYcaFOkP71P5NE/BB";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables targets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        VuforiaTrackable blueWheels  = targets.get(0); //load gears
        blueWheels.setName("Wheels");

        VuforiaTrackable blueLegos  = targets.get(2); //load gears
        blueLegos.setName("Legos");

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = (float)16.5 * mmPerInch;
        mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;

        OpenGLMatrix blueWheelsLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth/2, mmFTCFieldWidth/2 - (float)2082.8, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        blueWheels.setLocation(blueWheelsLocationOnField);
        RobotLog.ii(TAG, "Wheels=%s", format(blueWheelsLocationOnField));

        OpenGLMatrix blueLegosLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth/2, mmFTCFieldWidth/2 - (float)863.6, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        blueLegos.setLocation(blueLegosLocationOnField);
        RobotLog.ii(TAG, "Legos=%s", format(blueLegosLocationOnField));

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix //set up phone
                .translation(mmBotWidth/2,(float)44.45,200)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)blueWheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueLegos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        targets.activate();
    }

    public void loop()
    {
        /*for (VuforiaTrackable trackable : allTrackables) {
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }*/

        time = System.currentTimeMillis();

        heading = gyro.getHeading();
        trueHeading = degrees + heading;
        checkHeading();

        switch (control)
        {
            case 0: {//Set up for initial delay
                segmentTime = time;
                control = 1;
                telemetry.addData("Status", "Setting up start delay...");
                break;
            }
            case 1: {//Initial delay, set control to 2 to skip delay
                if (segmentTime + 10000 < time)
                    control = 2;
                telemetry.addData("Status", "Waiting to start...");
                break;
            }
            case 2: {//setup for first move
                segmentTime = time;
                control = 3;
                telemetry.addData("Status", "Preparing to move...");
                break;
            }
            case 3: {//move into position to shoot (timed move)
                if (navigateTime(90, .5, 1000))
                    control = 4;
                telemetry.addData("Status", "Moving for 1 seconds...");
                break;
            }
            case 4: {//setup for shooting
                allStop();
                segmentTime = time;
                control = 5;
                telemetry.addData("Status", "Preparing to shoot...");
                break;
            }
            case 5: {//shoot
                shoot();
                if (segmentTime + 2000 < time)
                    control = 6;
                telemetry.addData("Status", "Shooting for 2 seconds...");
                break;
            }
            case 6: {//setup for turn
                allStop();
                segmentTime = time;
                control = 7;
                telemetry.addData("Status", "Preparing to turn...");
                break;
            }
            case 7: {//turn 90 degrees left
                if (rotate('r', 80))
                    control = 8;
                telemetry.addData("Status", "Turning 90 degrees left...");
                break;
            }
            case 8: {//setup for move
                allStop();
                startDegrees += 90;
                control = 9;
                telemetry.addData("Status", "Preparing to move...");
                break;
            }
            case 9: {//move until target is visible
                navigate(45, .25);
                scan(allTrackables.get(0));
                if (((VuforiaTrackableDefaultListener) allTrackables.get(0).getListener()).isVisible())
                    control = 10;
                    segmentTime = time;
                telemetry.addData("Status", "Moving to find beacon...");
                break;
            }
            case 10: {//Wait for robot to completely stop
                allStop();
                scan(allTrackables.get(0));
                if (segmentTime + 1000 < time)
                    control = 11;
                break;

            }
            case 11: {//setup for lineup                                                            //coord of wheels
                allStop();
                scan(allTrackables.get(0));

                //Target x: 1512     Target y: -50
                double targetx = posx - 1512;
                double targety = posy - -50;
                startx = posx;
                starty = posy;

                targetDegrees = 180 - (int) ((180 / Math.PI) * (Math.atan(targety / targetx)));
                initialDegrees = targetDegrees;
                DbgLog.msg("START:");
                DbgLog.msg(targetDegrees + "");
                //targetDistance = Math.sqrt(Math.pow(Math.abs(targetx), 2) + Math.pow(Math.abs(targety), 2));
                control = 12;
                //telemetry.addData("targetx", targetx);
                //telemetry.addData("targety", targety);
                telemetry.addData("Status", "Beacon found! Calculating next position...");
                break;
            }
            case 12: {//attempt to lineup
                scan(allTrackables.get(0));
                navigateBlind(targetDegrees - 90, .25);
                if (sonar.getUltrasonicLevel() <= 15)
                    control = 14;
                if (segmentTime + 250 < time)
                    control = 13;
                telemetry.addData("Sonar", sonar.getUltrasonicLevel());
                //telemetry.addData("distance", targetDistance);
                telemetry.addData("degrees", targetDegrees);
                telemetry.addData("Status", "Lining up...");
                break;
            }
            case 13: {////check lineup, adjust if veering away from target
                scan(allTrackables.get(0));
                double targetx = posx - 1512;                                                       //fix coordinates
                double targety = posy - -50;

                targetDegrees = 180 - (int) ((180 / Math.PI) * (Math.atan(targety / targetx)));
                DbgLog.msg(targetDegrees + "");
                telemetry.addData("Status", "Checking lineup...");
                control = 12;
                segmentTime = time;
                break;
            }
            case 14: {//check beacon
                allStop();
                DbgLog.msg("END");
                if (color.blue() > 200)
                    pusher.setPosition(1);
                else
                    pusher.setPosition(0);
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
                break;
            }
            case 16: {//press a button
                if (navigateTime(90, .5, 500))
                    control = 17;
                break;
            }
            case 17: {//Move backwards until 50mm away from beacon
                navigateBlind(270, .35);
                if (sonar.getUltrasonicLevel() >= 50) {
                    control = 18;
                    segmentTime = time;
                }
                break;
            }
            case 18: {//Delay before next move
                allStop();
                if (segmentTime + 500 < time) {
                    segmentTime = time;
                    control = 19;
                }
                break;
            }
            case 19: {//Move until beacon is detected (Delay in detection to clear first beacon)
                scan(allTrackables.get(2));
                navigateBlind(5, .5);
                if ((segmentTime + 2000 <= time && sonar.getUltrasonicLevel() < 70 && sonar.getUltrasonicLevel() != -1) || ((VuforiaTrackableDefaultListener) allTrackables.get(2).getListener()).isVisible()) {
                    control = 20;
                    segmentTime = time;
                }
                break;
            }
            case 20: {//Stop to allow detection of picture, if picture is not found, go to alternate code
                allStop();
                scan(allTrackables.get(2));
                if (segmentTime + 500 < time) {
                    if (((VuforiaTrackableDefaultListener) allTrackables.get(2).getListener()).isVisible())
                        control = 21;
                    else {
                        control = 24;
                        segmentTime = time;
                    }
                }
                break;
            }
            case 21: {//Calculate initial movement angle                                            //coords of legos
                scan(allTrackables.get(2));
                double targetx = posx - 1512; //Position of beacon
                double targety = posy - 1172;
                startx = posx;
                starty = posy;

                targetDegrees = 180 - (int) ((180 / Math.PI) * (Math.atan(targety / targetx)));
                control = 22;
                segmentTime = time;
                break;
            }
            case 22: {//Move until beacon is within range
                scan(allTrackables.get(2));
                navigateBlind(targetDegrees - 90, .25);
                if (sonar.getUltrasonicLevel() <= 15)
                    control = 26;
                if (segmentTime + 125 < time)
                    control = 23;
                break;
            }
            case 23: {//Periodic angle adjustment for course correction                             //coord adjust
                scan(allTrackables.get(2));
                double targetx = posx - 1512;
                double targety = posy - 1172;

                targetDegrees = 180 - (int) ((180 / Math.PI) * (Math.atan(targety / targetx)));
                telemetry.addData("Status", "Checking lineup...");
                control = 22;
                segmentTime = time;
                break;
            }
            case 24: {//Alternate if beacon is not seen on first try
                scan(allTrackables.get(2));
                if (((VuforiaTrackableDefaultListener) allTrackables.get(2).getListener()).isVisible() || navigateTime(0, .25, 1000)) {
                    control = 25;
                    segmentTime = time;
                }
                break;
            }
            case 25: {//Delay before move to beacon
                scan(allTrackables.get(2));
                if (segmentTime + 1000 < time)
                    control = 21;
                allStop();
                break;
            }
            case 26: {
                allStop();
                if (color.blue() > 180)
                    pusher.setPosition(1);
                else
                    pusher.setPosition(0);
                control = 27;
                segmentTime = time;
                break;
            }
            case 27: {
                allStop();
                if (segmentTime + 1000 < time)
                    control = 28;
                break;
            }
            case 28: {
                if (navigateTime(90, .25, 1500))
                    control = 29;
                break;
            }
            case 29: {
                navigateBlind(270, .35);
                if (sonar.getUltrasonicLevel() >= 50)
                    control = 30;
                break;
            }
            default: {// hopefully this one runs too
                allStop();
            }
        }

        telemetry.addData("Timer", time - segmentTime);
        telemetry.addData("Control", control);
        telemetry.addData("Heading", trueHeading);
        telemetry.addData("Initial Degrees", initialDegrees);

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

        previousHeading = gyro.getHeading();
    }

    public boolean navigate(int deg, double power, double distance) //like unit circle, 90 forwards, 270 backwards
    {
        double x = Math.cos(deg * (Math.PI/180.0)), y = Math.sin(deg * (Math.PI/180.0));
        double targetx = distance * x + startx;
        double targety = distance * y + starty;
        if (targetx <= posx || targety >= posy)
        {
            double correction = correct();
            frontLeft.setPower((-(-y - x)/2) * power + correction);
            backLeft.setPower(((-y + x)/2) * power + correction);
            frontRight.setPower(((y - x)/2) * power + correction);
            backRight.setPower((-(y + x)/2) * power + correction);

            return false;
        }
        return true;
    }

    public boolean navigateTime(int deg, double power, long targetTime) //like unit circle, 90 forwards, 270 backwards
    {
        double x = Math.cos(deg * (Math.PI/180.0)), y = Math.sin(deg * (Math.PI/180.0));

        if (segmentTime + targetTime > time)
        {
            double correction = correct();
            frontLeft.setPower((-(-y - x)/2) * power + correction);
            backLeft.setPower(((-y + x)/2) * power + correction);
            frontRight.setPower(((y - x)/2) * power + correction);
            backRight.setPower((-(y + x)/2) * power + correction);

            return false;
        }
        return true;
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

    public void navigate(int deg, double power)
    {
        double x = Math.cos(deg * (Math.PI/180.0)), y = Math.sin(deg * (Math.PI/180.0));

        double correction = correct();
        frontLeft.setPower((-(-y - x)/2) * power + correction);
        backLeft.setPower(((-y + x)/2) * power + correction);
        frontRight.setPower(((y - x)/2) * power + correction);
        backRight.setPower((-(y + x)/2) * power + correction);
    }

    public boolean rotate(char direction, int deg)
    {
        target = deg;
        if (direction == 'r' && trueHeading - target < 0)
        {
            frontRight.setPower(-.15);
            frontLeft.setPower(-.15);
            backRight.setPower(-.15);
            backLeft.setPower(-.15);
            return false;
        }
        else if (direction == 'l' && trueHeading - target > 0)
        {
            frontRight.setPower(.15);
            frontLeft.setPower(.15);
            backRight.setPower(.15);
            backLeft.setPower(.15);
            return false;
        }
        return true;
    }

    public void checkHeading()
    {
        if (previousHeading - heading > 300)
            degrees += 360;
        else if (heading - previousHeading > 300)
            degrees -= 360;
    }

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

    public double correct()
    {
        if (trueHeading > startDegrees + 3)
            return .1;
        if (trueHeading < startDegrees - 3)
            return -.1;
        return 0;
    }

    public boolean shoot() //Waiting for launcher to be built, no code implemented
    {
        return true;
    }

    public void scan(VuforiaTrackable t) //for t, use allTrackables.get(). 0 is Wheels, 1 is Tools, 2 is Legos, 3 is Gears
    {
        telemetry.addData(t.getName(), ((VuforiaTrackableDefaultListener) t.getListener()).isVisible() ? "Visible" : "Not Visible");    //

        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) t.getListener()).getUpdatedRobotLocation();
        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;
        }
    }
}
