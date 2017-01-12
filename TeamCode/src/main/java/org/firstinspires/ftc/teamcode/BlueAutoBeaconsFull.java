package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous (name = "Blue pos 1: Shoot 2/Press 2 SPEED", group = "Blue Autonomous")
public class BlueAutoBeaconsFull extends OpMode {
    //Removing all possible delays and speeding up motors
    public final int VERSION = 17;

    int target, startDegrees, targetDegrees, shooterStartPos, sideOfLine, beaconState, target2 = 0, pushCheck = 0, rotateDegrees = 0;
    int[] beaconPos1 = {1440, -485}, beaconPos2 = {1440, 740};//{x, y}
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor shooter;
    DcMotor sweeper;
    GyroSensor gyro;
    float robotBearing;
    boolean pushable = false, shootAbort = false;

    Long time, startTime, segmentTime;
    VuforiaLocalizer vuforia;
    List<VuforiaTrackable> allTrackables;
    double posx, posy, startx, starty, targetDistance;
    float mmFTCFieldWidth;
    ColorSensor color;
    ColorSensor line;
    UltrasonicSensor sonar, spareSonar;
    Servo pusher;

    public static final String TAG = "Vuforia Sample";

    public enum RobotSteps {INIT_START, DELAY, INITIAL_MOVE, INIT_MOVE_TO_BEACON, MOVE_TO_BEACON,
        INIT_ALIGN, ALIGN, INIT_MOVE_TO_PUSH_POS, MOVE_TO_PUSH_POS, INIT_REALIGN, REALIGN, INIT_SCAN, SCAN, INIT_PUSH,
        PUSH, CHECK_PUSH, REVERSE, REREALIGN, INIT_MOVE_TO_BEACON2, MOVE_TO_BEACON2, COMPLETE, RANGE_CHECK, PRESCAN,
        TURN_TO_SHOOT, INIT_MOVE_TO_SHOOT, MOVE_TO_SHOOT, INIT_SHOOT, SHOOT, SWEEPER_MOVE_BACKWARD, SWEEPER_MOVE_FORWARD, SHOOT_TWO};
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
        spareSonar = hardwareMap.ultrasonicSensor.get("sonar2");
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

        VuforiaTrackable blueWheels  = targets.get(0); //load wheels
        blueWheels.setName("Wheels");

        VuforiaTrackable blueLegos  = targets.get(2); //load legos
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
                .translation(mmBotWidth/2,(float)44.45 + 175,200)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, -90, -90, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)blueWheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueLegos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        targets.activate();
        telemetry.addData("Version", VERSION);
        telemetry.update();
    }

    public void loop()
    {
        //Constantly updating variables go here
        time = System.currentTimeMillis();

        int heading = gyro.getHeading() - rotateDegrees;
        if (heading + rotateDegrees > 180)
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
                if (segmentTime < time) {
                    control = RobotSteps.INIT_MOVE_TO_BEACON;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Waiting for 5 seconds...");
                break;
            }
            case INITIAL_MOVE: {
                if (navigateTime(0, .6, 850, heading)) {
                    control = RobotSteps.MOVE_TO_BEACON;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Moving to shooting position...");
                break;
            }
            case INIT_MOVE_TO_BEACON: {
                scan(allTrackables.get(target2));
                if (segmentTime < time) {
                    control = RobotSteps.MOVE_TO_BEACON;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Preparing to move to beacon...");
                break;
            }
            case MOVE_TO_BEACON: {
                boolean isVisible = scan(allTrackables.get(target2));
                if (!isVisible)
                    navigateBlind(55, .7, heading);
                if (isVisible && posx > beaconPos1[0]) {
                    navigateBlind(90, .7, heading);
                }
                if ((segmentTime + 1000 < time && sonar.getUltrasonicLevel() > 0 && sonar.getUltrasonicLevel() < 45) || line.alpha() > 20) {
                    segmentTime = time;
                    control = RobotSteps.INIT_ALIGN;
                    allStop();
                }
                telemetry.addData("Status", "Moving to beacon...");
                break;
            }
            case INIT_ALIGN: {
                boolean isVisible = scan(allTrackables.get(target2));
                int y = 0;
                if (target2 == 0)
                    y = beaconPos1[1];
                if (target2 == 2)
                    y = beaconPos2[1];
                if (realign(heading) && isVisible) {
                    scan(allTrackables.get(3));
                    if (line.alpha() > 10)
                        sideOfLine = 0;//on target
                    else if (posy > y)
                        sideOfLine = -1;//left of target
                    else if (posy < y)
                        sideOfLine = 1;//right of target
                    control = RobotSteps.INIT_MOVE_TO_PUSH_POS;
                    segmentTime = time;
                } else if (segmentTime + 1000 < time) {
                    sideOfLine = 1;
                    control = RobotSteps.INIT_MOVE_TO_PUSH_POS;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Realigning...");
                break;
            }
            case INIT_MOVE_TO_PUSH_POS: {
                allStop();
                scan(allTrackables.get(target2));
                pusher.setPosition(.5);
                if (segmentTime + 100 < time)
                    control = RobotSteps.MOVE_TO_PUSH_POS;
                break;
            }
            case MOVE_TO_PUSH_POS: {
                scan(allTrackables.get(target2));
                navigateBlind(90, .5, heading);
                if (sonar.getUltrasonicLevel() < 25)
                    allStop();
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
                if (segmentTime + 3000 < time) {
                    target2 = 1;
                    segmentTime = time;
                }

                int y = 0;
                if (target2 == 0)
                    y = beaconPos1[1];
                if (target2 == 2)
                    y = beaconPos2[1];
                if (realign(heading)) {
                    if (isVisible) {
                        if (line.alpha() > 20)
                            sideOfLine = 0;//on target
                        else if (posy > y)
                            sideOfLine = -1;//left of target
                        else if (posy < y)
                            sideOfLine = 1;//right of target
                    }
                    else
                        sideOfLine = 1;
                    control = RobotSteps.REALIGN;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Realigning...");
                break;
            }
            case REALIGN: {
                boolean isVisible = scan(allTrackables.get(target2));
                int y = 0;
                if (target2 == 0)
                    y = beaconPos1[1];
                if (target2 == 2)
                    y = beaconPos2[1];
                if (sideOfLine == 0) {
                    control = RobotSteps.PRESCAN;
                    segmentTime = time;
                }
                else {
                    if (isVisible)
                        allStop();
                    if (sideOfLine == -1) {//left
                        navigateBlind(180, .35, heading);
                        if (posy + 20 < y)
                            sideOfLine = 1;
                        if (line.alpha() > 20)
                            sideOfLine = 0;
                    }
                    if (sideOfLine == 1) {//right
                        navigateBlind(0, .35, heading);
                        if (posy - 20 > y)
                            sideOfLine = -1;
                        if (line.alpha() > 20)
                            sideOfLine = 0;
                    }
                }
                telemetry.addData("Status", "Finding line...");
                break;
            }
            case PRESCAN: {
                if (sonar.getUltrasonicLevel() > 20) {
                    navigateBlind(90, .5, heading);
                    allStop();
                } else if (sonar.getUltrasonicLevel() < 15) {
                    navigateBlind(270, .5, heading);
                    allStop();
                } else {
                    control = RobotSteps.INIT_SCAN;
                }
            }
            case INIT_SCAN: {
                allStop();
                if (realign(heading)) {
                    if (line.alpha() < 10)
                        control = RobotSteps.INIT_REALIGN;
                    else
                        control = RobotSteps.SCAN;
                }
                break;
            }
            case SCAN: {
                if (color.blue() < color.red()) {
                    pusher.setPosition(1);
                    beaconState = 1;
                } else if (color.blue() > color.red()) {
                    pusher.setPosition(0);
                    beaconState = -1;
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
            }
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
            }
            case PUSH: {
                if (segmentTime + 500 < time)
                    control = RobotSteps.REVERSE;
                if (beaconState == -1) {
                    frontLeft.setPower(.2);
                    frontRight.setPower(.2);
                    backLeft.setPower(-.3);
                    backRight.setPower(-.3);
                } else if (beaconState == 1) {
                    frontLeft.setPower(.3);
                    frontRight.setPower(.3);
                    backLeft.setPower(-.2);
                    backRight.setPower(-.2);
                }
                break;
            }
            case REVERSE: {
                navigateBlind(270, .5, heading);
                if (sonar.getUltrasonicLevel() > 25) {
                    control = RobotSteps.REREALIGN;
                    if (target2 == 2)
                        control = RobotSteps.TURN_TO_SHOOT;
                    if (target2 == 0)
                        target2 = 2;
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
                if (segmentTime < time) {
                    control = RobotSteps.MOVE_TO_BEACON2;
                    segmentTime = time;
                }
                break;
            }
            case MOVE_TO_BEACON2: {
                boolean isVisible = scan(allTrackables.get(target2));
                if (segmentTime + 500 > time)
                    navigateBlind(5, .5, heading);
                else
                    navigateBlind(5, .35, heading);

                if ((isVisible && posx > 1170) || (line.alpha() > 10 && segmentTime + 800 < time) || (spareSonar.getUltrasonicLevel() <= 65 && segmentTime + 800 < time)) {
                    control = RobotSteps.INIT_ALIGN;
                    allStop();
                }
                break;
            }
            case TURN_TO_SHOOT: {
                rotateDegrees = 40;
                if (realign(heading)) {
                    control = RobotSteps.INIT_MOVE_TO_SHOOT;
                    allStop();
                }
                break;
            }
            case INIT_MOVE_TO_SHOOT: {
                segmentTime = time;
                control = RobotSteps.MOVE_TO_SHOOT;
                segmentTime = time;
                break;
            }
            case MOVE_TO_SHOOT: {
                navigateBlind(180, .5, heading);
                if (segmentTime + 1350 < time) {
                    control = RobotSteps.INIT_SHOOT;
                    allStop();
                }
                break;
            }
            case INIT_SHOOT: {
                allStop();
                shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooter.setTargetPosition(-1340);
                shooterStartPos = 0;
                if ((30000 - (time - startTime))/1000 > .5)
                    control = RobotSteps.SHOOT;
                else {
                    control = RobotSteps.COMPLETE;
                    shootAbort = true;
                }
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
                    control = RobotSteps.SWEEPER_MOVE_FORWARD;
                }
                break;
            }
            case SWEEPER_MOVE_FORWARD: {//swpr.mov -> > var(.7) -- pos+
                sweeper.setPower(.7);
                if (segmentTime + 1300 < time)
                {
                    if ((30000 - (time - startTime))/1000 > .5)
                        control = RobotSteps.SHOOT_TWO;
                    else {
                        control = RobotSteps.COMPLETE;
                        shootAbort = true;
                    }
                    shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    sweeper.setPower(0);
                    shooter.setTargetPosition(-1340);
                    segmentTime = time;
                }
                break;
            }
            case SHOOT_TWO: {//shoot^2
                if (!shoot() ) {
                    control = RobotSteps.COMPLETE;
                    segmentTime = time;
                }
                break;
            }
            default: {//Hopefully this only runs when program ends
                allStop();
                if (shootAbort)
                    telemetry.addData("WARNING", "Could not shoot due to lack of time!");
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

    public boolean realign (int h)
    {
        if (Math.abs(h - startDegrees) < 10)
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