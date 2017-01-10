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
 * Created by bense on 12/6/2016.
 */
@Autonomous (name = "Red pos 1: Shoot 2/Press 1.5 EX", group = "Red Autonomous")
@Disabled
public class RedAutoBeacons2_EX extends OpMode {
    public final int VERSION = 10;

    public final int NUM_BEACONS = 2;
    int target, startDegrees, targetDegrees, shooterStartPos, sideOfLine, beaconState, target2 = 3, pushCheck = 0;
    int[] beaconPos1 = {-30, 1465}, beaconPos2 = {1170, 1465};//{x, y}
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor shooter;
    DcMotor sweeper;
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

    public static final String TAG = "Vuforia Sample";

    public enum RobotSteps {INIT_START, DELAY, MOVE_TO_SHOOT, INIT_SHOOT, SHOOT, INIT_MOVE_TO_BEACON, MOVE_TO_BEACON,
        INIT_ALIGN, ALIGN, INIT_MOVE_TO_PUSH_POS, MOVE_TO_PUSH_POS, INIT_REALIGN, REALIGN, INIT_SCAN, SCAN, INIT_PUSH,
        PUSH, CHECK_PUSH, REVERSE, REREALIGN, INIT_MOVE_TO_BEACON2, MOVE_TO_BEACON2, COMPLETE, RANGE_CHECK, SWEEPER_MOVE_BACKWARD, SWEEPER_MOVE_FORWARD, SHOOT_DOS};
    RobotSteps control = RobotSteps.INIT_START;
    OpenGLMatrix lastLocation = null;
    String loopNumber;
    public void init()
    {
        sweeper = hardwareMap.dcMotor.get("sweeper");
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
        telemetry.addData("Version", VERSION);
        telemetry.update();
    }

    public void loop()
    {
        //Constantly updating variables go here
        time = System.currentTimeMillis();

        int heading = gyro.getHeading();
        if (heading > 180)
            heading -= 360;

        //Main switch for controlling robot
        switch (control)
        {
            case INIT_START: {
                startTime = System.currentTimeMillis();
                segmentTime = time;
                control = RobotSteps.DELAY;
                startDegrees = heading;
                telemetry.addData("Status", "Setting up delay...");
                break;
            }
            case DELAY: {//No Delay
                if (segmentTime + 0 < time) {
                    control = RobotSteps.MOVE_TO_SHOOT;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Waiting for 5 seconds...");
                break;
            }
            case MOVE_TO_SHOOT: {
                if (navigateTime(180, .6, 850, heading))
                    control = RobotSteps.INIT_SHOOT;
                telemetry.addData("Status", "Moving to shooting position...");
                break;
            }
            case INIT_SHOOT: {
                allStop();
                shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooter.setTargetPosition(-1340);
                shooterStartPos = 0;
                control = RobotSteps.SHOOT;
                telemetry.addData("Status", "Setting up shooter...");
                break;
            }
            case SHOOT: {
                if (!shoot()) {
                    segmentTime = time;
                    control = RobotSteps.SWEEPER_MOVE_BACKWARD;
                }
                telemetry.addData("Status", "Shooting particle...");
                break;
            }
            case SWEEPER_MOVE_BACKWARD: {//swpr.mov -> < var(-.5)
                sweeper.setPower(-.5);
                if (segmentTime + 70 < time) {
                    sweeper.setPower(0);
                    segmentTime = time;
                    control = RedAutoBeacons2_EX.RobotSteps.SWEEPER_MOVE_FORWARD;
                }
                break;
            }
            case SWEEPER_MOVE_FORWARD: {//swpr.mov -> > var(.7) -- pos+
                sweeper.setPower(.7);
                if (segmentTime + 1300 < time)
                {
                    control = RedAutoBeacons2_EX.RobotSteps.SHOOT_DOS;
                    shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    sweeper.setPower(0);
                    shooter.setTargetPosition(-1340);
                }
                break;

            }
            case SHOOT_DOS: {//shoot^2
                if (!shoot() ) {
                    control = RedAutoBeacons2_EX.RobotSteps.INIT_MOVE_TO_BEACON;
                    segmentTime = time;
                }
                break;

            }
            case INIT_MOVE_TO_BEACON: {
                scan(allTrackables.get(target2));
                if (segmentTime + 500 < time) {
                    control = RobotSteps.MOVE_TO_BEACON;
                }
                telemetry.addData("Status", "Preparing to move to beacon...");
                break;
            }
            case MOVE_TO_BEACON: {
                boolean isVisible = scan(allTrackables.get(target2));
                if (!isVisible)
                    navigateBlind(135, .35, heading);
                if (isVisible && posx > beaconPos1[0]) {
                    navigateBlind(90, .3, heading);
                }
                if ((sonar.getUltrasonicLevel() > 0 && sonar.getUltrasonicLevel() < 45) || line.alpha() > 20) {
                    segmentTime = time;
                    control = RobotSteps.INIT_ALIGN;
                    allStop();
                }
                telemetry.addData("Status", "Moving to beacon...");
                break;
            }
            case INIT_ALIGN: {
                boolean isVisible = scan(allTrackables.get(target2));
                int x = 0;
                if (target == 3)
                    x = beaconPos1[0];
                if (target == 1)
                    x = beaconPos2[0];
                if (realign(heading) && isVisible) {
                    scan(allTrackables.get(3));
                    if (line.alpha() > 10)
                        sideOfLine = 0;//on target
                    else if (posx < x)
                        sideOfLine = -1;//left of target
                    else if (posx > x)
                        sideOfLine = 1;//right of target
                    control = RobotSteps.ALIGN;
                    segmentTime = time;
                } else if (segmentTime + 1000 < time) {
                    sideOfLine = 1;
                    control = RobotSteps.ALIGN;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Realigning...");
                break;
            }
            case ALIGN: {
                scan(allTrackables.get(target2));
                int x = 0;
                if (target == 3)
                    x = beaconPos1[0];
                if (target == 1)
                    x = beaconPos2[0];
                if (sideOfLine == 0) {
                    control = RobotSteps.INIT_MOVE_TO_PUSH_POS;
                    segmentTime = time;
                } else {
                    if (sideOfLine == -1) {
                        navigateBlind(180, .3, heading);
                        if (posx - 20 > x) {
                            sideOfLine = 1;
                            segmentTime = time;
                        }
                        if (line.alpha() > 20)
                            sideOfLine = 0;
                    }
                    if (sideOfLine == 1) {
                        navigateBlind(0, .3, heading);
                        if (posx + 20 < x) {
                            sideOfLine = -1;
                            segmentTime = time;
                        }
                        if (line.alpha() > 20)
                            sideOfLine = 0;
                    }
                }
                telemetry.addData("Status", "Finding line...");
                break;
            }
            case INIT_MOVE_TO_PUSH_POS: {
                allStop();
                scan(allTrackables.get(target));
                pusher.setPosition(.5);
                if (segmentTime + 500 < time)
                    control = RobotSteps.MOVE_TO_PUSH_POS;
                break;
            }
            case MOVE_TO_PUSH_POS: {
                scan(allTrackables.get(target2));
                navigateBlind(90, .3, heading);
                if (sonar.getUltrasonicLevel() > 0 && sonar.getUltrasonicLevel() < 20 + (heading * .5)) {
                    allStop();
                    control = RobotSteps.RANGE_CHECK;
                }
                break;
            }
            case RANGE_CHECK: {
                scan(allTrackables.get(target2));
                if (sonar.getUltrasonicLevel() <= 15 && segmentTime + 500 > time) {
                    navigateBlind(270, .3, heading);
                } else {
                    control = RobotSteps.INIT_REALIGN;
                    allStop();
                    segmentTime = time;
                }
                break;
            }
            case  INIT_REALIGN: {
                boolean isVisible = scan(allTrackables.get(target2));
                pusher.setPosition(0);
                if (segmentTime + 5000 < time) {
                    target2 = 1;
                    segmentTime = time;
                }

                int x = 0;
                if (target == 3)
                    x = beaconPos1[0];
                if (target == 1)
                    x = beaconPos2[0];
                if (isVisible && realign(heading)) {
                    scan(allTrackables.get(3));
                    if (line.alpha() > 10)
                        sideOfLine = 0;//on target
                    else if (posx < x)
                        sideOfLine = -1;//left of target
                    else if (posx > x)
                        sideOfLine = 1;//right of target
                    control = RobotSteps.REALIGN;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Realigning...");
                break;
            }
            case REALIGN: {
                scan(allTrackables.get(target2));
                int x = 0;
                if (target == 3)
                    x = beaconPos1[0];
                if (target == 1)
                    x = beaconPos2[0];
                if (sideOfLine == 0) {
                    control = RobotSteps.INIT_SCAN;
                    segmentTime = time;
                }
                else {
                    allStop();
                    if (sideOfLine == -1) {
                        navigateBlind(180, .3, heading);
                        if (posx - 20 > x)
                            sideOfLine = 1;
                        if (line.alpha() > 20)
                            sideOfLine = 0;
                    }
                    if (sideOfLine == 1) {
                        navigateBlind(0, .3, heading);
                        if (posx + 20 < x)
                            sideOfLine = -1;
                        if (line.alpha() > 20)
                            sideOfLine = 0;
                    }
                }
                telemetry.addData("Status", "Finding line...");
                break;
            }
            case INIT_SCAN: {
                allStop();
                if (realign(heading))
                    control = RobotSteps.SCAN;
                break;
            }
            case SCAN: {
                if (color.blue() < color.red()) {
                    pusher.setPosition(0);
                    beaconState = -1;
                } else if (color.blue() > color.red()) {
                    pusher.setPosition(1);
                    beaconState = 1;
                } else {
                    pusher.setPosition(.5);
                    beaconState = 0;
                }
                control = RobotSteps.INIT_PUSH;
                segmentTime = time;
                break;
            }
            case INIT_PUSH: {
                allStop();
                if (segmentTime + 500 < time) {
                    control = RobotSteps.PUSH;
                    segmentTime = time;
                    if (sonar.getUltrasonicLevel() > 20)
                        pushCheck = 1;
                    if (sonar.getUltrasonicLevel() < 15)
                        pushCheck = -1;
                }
                break;
            }/*
            case CHECK_PUSH: {
                if (sonar.getUltrasonicLevel() > 15 && sonar.getUltrasonicLevel() < 20)
                    pushCheck = 0;

                if (pushCheck == 1) {
                    navigateBlind(90, .35, heading);
                } else if (pushCheck == -1) {
                    navigateBlind(270, .35, heading);
                } else if (pushCheck == 0) {
                    allStop();
                    control = RobotSteps.PUSH;
                    segmentTime = time;
                }
                break;
            }*/
            case PUSH: {
                if (segmentTime + 500 < time)
                    control = RobotSteps.REVERSE;
                if (beaconState == -1) {
                    frontLeft.setPower(.1);
                    frontRight.setPower(.1);
                    backLeft.setPower(-.4);
                    backRight.setPower(-.4);
                } else if (beaconState == 1) {
                    frontLeft.setPower(.4);
                    frontRight.setPower(.4);
                    backLeft.setPower(-.1);
                    backRight.setPower(-.1);
                }
                break;
            }
            case REVERSE: {
                navigateBlind(270, .5, heading);
                if (sonar.getUltrasonicLevel() > 35) {
                    control = RobotSteps.REREALIGN;
                    if (target2 == 1)
                        control = RobotSteps.COMPLETE;
                    if (target2 == 3)
                        target2 = 1;
                    segmentTime = time;
                }
                break;
            }
            case REREALIGN: {
                if (realign(heading)) {
                    control = RobotSteps.INIT_MOVE_TO_BEACON2;
                    segmentTime = time;
                }
                break;
            }
            case INIT_MOVE_TO_BEACON2: {
                pusher.setPosition(0);
                scan(allTrackables.get(target2));
                if (segmentTime + 500 < time) {
                    control = RobotSteps.MOVE_TO_BEACON2;
                    segmentTime = time;
                }
                break;
            }
            case MOVE_TO_BEACON2: {
                boolean isVisible = scan(allTrackables.get(target2));
                navigateBlind(175, .35, heading);

                if ((isVisible && posx > 1170) || (line.alpha() > 10 && segmentTime + 1500 < time)) {
                    control = RobotSteps.INIT_ALIGN;
                    allStop();
                }
                break;
            }
            default: {//Hopefully this only runs when program ends
                allStop();
                telemetry.addData("Status", "Switch is in default. Waiting for autonomous to end...");
            }
        }

        //All telemetry goes here
        telemetry.addData("Time Left", Math.ceil((30000 - (time - startTime))/1000));
        telemetry.addData("Segment Time", time - segmentTime);
        if (sideOfLine == -1)
            telemetry.addData("Side of Line", "Left");
        else if (sideOfLine == 0)
            telemetry.addData("Side of Line", "On Line");
        else
            telemetry.addData("Side of Line", "Right");
        telemetry.addData("Control", control);
        telemetry.addData("Heading", heading);
        if (color.blue() > color.red())
            telemetry.addData("Color", "Blue");
        else if (color.blue() < color.red())
            telemetry.addData("Color", "Red");
        else
            telemetry.addData("Color", "Unknown");
        telemetry.addData("Target2", target2);
        if (lastLocation != null) {//position output
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

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public double correct(int h)
    {
        return (h * .01)/2;
    }

    public boolean shoot()
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

    public boolean scan(VuforiaTrackable t) //for t, use allTrackables.get(). 0: Wheels, 1: Tools, 2: Legos, 3: Gears
    {
        telemetry.addData(t.getName(), ((VuforiaTrackableDefaultListener) t.getListener()).isVisible() ? "Visible" : "Not Visible");    //

        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) t.getListener()).getUpdatedRobotLocation();
        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;
            return true;
        }
        return false;
    }
    /*
    public boolean realign (int h)
    {
        if (h + 6 < startDegrees) {
            frontRight.setPower(-.08);
            frontLeft.setPower(-.08);
            backRight.setPower(-.08);
            backLeft.setPower(-.08);
            segmentTime = time;
        } else if (h - 6 > startDegrees) {
            frontRight.setPower(.08);
            frontLeft.setPower(.08);
            backRight.setPower(.08);
            backLeft.setPower(.08);
            segmentTime = time;
        } else {
            allStop();
            if (segmentTime + 500 < time)
                return true;
        }
        return false;
    }*/
    public boolean realign (int h)
    {
        allStop();
        if (h + 3 < startDegrees) {
            frontRight.setPower(-.08);
            frontLeft.setPower(-.08);
            backRight.setPower(-.08);
            backLeft.setPower(-.08);
            segmentTime = time;
        } else if (h - 3 > startDegrees) {
            frontRight.setPower(.08);
            frontLeft.setPower(.08);
            backRight.setPower(.08);
            backLeft.setPower(.08);
            segmentTime = time;
        } else {
            allStop();
            if (segmentTime + 500 < time)
                return true;
        }
        return false;
    }
}