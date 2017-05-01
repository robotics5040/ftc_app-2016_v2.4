package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Trackable;
import com.vuforia.TrackableSource;
import com.vuforia.VuforiaBase;

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
@Autonomous (name = "Red pos 1: Shoot 2/Press 2/Park", group = "Red Autonomous")
public class RedAutoBeaconsFull4 extends OpMode {
    public final int VERSION = 21;

    public final int NUM_BEACONS = 2;
    int target, startDegrees, targetDegrees, shooterStartPos, sideOfLine, beaconState, target2 = 3, pushCheck = 0, shooterDeg = 0;
    int[] beaconPos1 = {-75, 1465}, beaconPos2 = {1110, 1465};//{x, y}
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor shooter;
    DcMotor sweeper;
    GyroSensor gyro;
    float robotBearing;
    boolean pushable = false, guessing = false;

    Long time, startTime, segmentTime;
    VuforiaLocalizer vuforia;
    List<VuforiaTrackable> allTrackables;
    double posx, posy, startx, starty, targetDistance;
    float mmFTCFieldWidth;
    ColorSensor color;
    ColorSensor line;
    ColorSensor lineLeft;
    ColorSensor lineRight;
    ModernRoboticsI2cRangeSensor sonar;
    //UltrasonicSensor spareSonar;
    I2cDevice spareSonar;
    I2cDeviceSynch spareSonarReader;
    byte[] spareSonarCache;
    Servo pusher;

    public static final String TAG = "Vuforia Sample";

    public enum RobotSteps {INIT_START, DELAY, MOVE_TO_SHOOT, INIT_SHOOT, SHOOT, INIT_MOVE_TO_BEACON, MOVE_TO_BEACON,
        INIT_ALIGN, ALIGN, INIT_MOVE_TO_PUSH_POS, MOVE_TO_PUSH_POS, INIT_REALIGN, REALIGN, INIT_SCAN, SCAN, INIT_PUSH,
        PUSH, CHECK_PUSH, REVERSE, REREALIGN, INIT_MOVE_TO_BEACON2, MOVE_TO_BEACON2, COMPLETE, RANGE_CHECK, PRESCAN,
        SWEEPER_MOVE_BACKWARD, SWEEPER_MOVE_FORWARD, SHOOT_TWO, MOVE_TO_PARK, CHECK_WALL, BACK_UP, B2_LINEUP_CHECK, FINAL_ALIGN};
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
        sonar = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sonar");
        spareSonar = hardwareMap.i2cDevice.get("wallSonar");
        spareSonarReader = new I2cDeviceSynchImpl(spareSonar, I2cAddr.create8bit(0x2a), false);
        spareSonarReader.engage();

        color = hardwareMap.colorSensor.get("color");

        line = hardwareMap.colorSensor.get("line");
        line.setI2cAddress(I2cAddr.create8bit(0x42));

        lineLeft = hardwareMap.colorSensor.get("lineLeft");
        lineLeft.setI2cAddress(I2cAddr.create8bit(0x40));

        lineRight = hardwareMap.colorSensor.get("lineRight");
        lineRight.setI2cAddress(I2cAddr.create8bit(0x3e));

        pusher = hardwareMap.servo.get("pusher");
        pusher.setPosition(.25);
        color.enableLed(false);
        line.enableLed(true);
        lineLeft.enableLed(true);
        lineRight.enableLed(true);

        gyro.calibrate();

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        sweeper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        if (color.alpha() == 255)
            telemetry.addData("WARNING", "Main color sensor not responding!");
        if (lineLeft.alpha() == 255)
            telemetry.addData("WARNING", "Left line sensor not responding!");
        if (line.alpha() == 255)
            telemetry.addData("WARNING", "Center line sensor not responding!");
        if (lineRight.alpha() == 255)
            telemetry.addData("WARNING", "Right line sensor not responding!");
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
                if (segmentTime < time) {
                    control = RobotSteps.MOVE_TO_SHOOT;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Waiting for 5 seconds...");
                break;
            }
            case MOVE_TO_SHOOT: {
                pusher.setPosition(.25);
                if (navigateTime(180, .6, 1150, heading))
                    control = RobotSteps.INIT_SHOOT;
                telemetry.addData("Status", "Moving to shooting position...");
                break;
            }
            case INIT_SHOOT: {
                allStop();
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
                    shooterDeg = shooter.getCurrentPosition();
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
                    control = RobotSteps.SHOOT_TWO;

                    sweeper.setPower(0);
                    shooter.setTargetPosition(-1340 + shooterDeg);
                    segmentTime = time;
                }
                break;
            }
            case SHOOT_TWO: {//shoot^2
                if (!shoot() ) {
                    control = RobotSteps.INIT_MOVE_TO_BEACON;
                    segmentTime = time;
                }
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
                double pow;
                if (segmentTime + 1000 > time)
                    pow = .5;
                else
                    pow = .6;

                if (!isVisible)
                    navigateBlind(135, pow, heading);
                if (isVisible && posx > beaconPos1[0]) {
                    navigateBlind(90, pow, heading);
                }
                if ((sonar.cmUltrasonic() > 0 && sonar.cmUltrasonic() < 40) || line.alpha() > 20) {
                    segmentTime = time;
                    control = RobotSteps.CHECK_WALL;
                    allStop();
                }
                telemetry.addData("Status", "Moving to beacon...");
                break;
            }
            case CHECK_WALL: {
                allStop();
                if (segmentTime + 500 < time) {
                    if (sonar.cmUltrasonic() > 0 && sonar.cmUltrasonic() < 40) {
                        control = RobotSteps.INIT_ALIGN;
                    } else {
                        control = RobotSteps.MOVE_TO_BEACON;
                    }
                }
                break;
            }
            case INIT_ALIGN: {
                boolean isVisible = scan(allTrackables.get(target2));
                int x = 0;
                if (target2 == 3)
                    x = beaconPos1[0];
                if (target2 == 1)
                    x = beaconPos2[0];
                if (realign(heading) && isVisible) {
                    if (line.alpha() > 20)
                        sideOfLine = 0;//on target
                    else if (posx < x)
                        sideOfLine = -1;//left of target
                    else if (posx > x)
                        sideOfLine = 1;//right of target
                    control = RobotSteps.ALIGN;
                    segmentTime = time;
                } else if (segmentTime + 1000 < time) {
                    //if (target2 == 3)
                    //    sideOfLine = -1;
                    //else
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
                if (target2 == 3)
                    x = beaconPos1[0];
                if (target2 == 1)
                    x = beaconPos2[0];
                if (sideOfLine == 0) {
                    control = RobotSteps.INIT_MOVE_TO_PUSH_POS;
                    segmentTime = time;
                } else {
                    if (sideOfLine == -1) {
                        navigateBlind(180, .35, heading);
                        if (posx - 20 > x) {
                            sideOfLine = 1;
                            segmentTime = time;
                        }
                        if (line.alpha() > 20)
                            sideOfLine = 0;
                    }
                    if (sideOfLine == 1) {
                        navigateBlind(0, .35, heading);
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
                //Turn off Camera
                if (line.alpha() > 20)
                    sideOfLine = 0;
                else if (lineLeft.alpha() > 20)
                    sideOfLine = -1;
                else if (lineRight.alpha() > 20)
                    sideOfLine = 1;
                if (segmentTime + 1000 < time)
                    control = RobotSteps.MOVE_TO_PUSH_POS;
                break;
            }
            case MOVE_TO_PUSH_POS: {
                allStop();
                if (sideOfLine == 0) {
                    if (sonar.cmUltrasonic() > 17) {
                        navigateBlind(90, .6, heading);
                    } else {
                        control = RobotSteps.FINAL_ALIGN;
                        guessing = true;
                    }
                } else if (sideOfLine == 1) {
                    if (sonar.cmUltrasonic() > 17)
                        navigateBlind(135, .35, heading);
                    else
                        navigateBlind(180, .35, heading);
                } else if (sideOfLine == -1) {
                    if (sonar.cmUltrasonic() > 17)
                        navigateBlind(45, .35, heading);
                    else
                        navigateBlind(0, .35, heading);
                }
                if (lineLeft.alpha() > 20) {
                    sideOfLine = -1;
                } else if (line.alpha() > 20) {
                    sideOfLine = 0;
                } else if (lineRight.alpha() > 20) {
                    sideOfLine = 1;
                }
                break;
            }
            case FINAL_ALIGN: {
                allStop();
                if (sideOfLine == 0) {
                    control = RobotSteps.PRESCAN;
                } else if (sideOfLine == -1) {
                    navigateBlind(0, .33, heading);
                } else if (sideOfLine == 1) {
                    navigateBlind(180, .33, heading);
                }
                if (lineLeft.alpha() > 20) {
                    guessing = false;
                    sideOfLine = -1;
                } else if (line.alpha() > 20) {
                    guessing = false;
                    sideOfLine = 0;
                } else if (lineRight.alpha() > 20) {
                    guessing = false;
                    sideOfLine = 1;
                }
                telemetry.addData("Guessing", guessing);
                break;
            }
            /*case RANGE_CHECK: {
                scan(allTrackables.get(target2));
                if (sonar.cmUltrasonic() <= 15 && segmentTime + 500 > time) {
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
                pusher.setPosition(.25);
                if (segmentTime + 3000 < time) {
                    target2 = 1;
                    segmentTime = time;
                }

                int x = 0;
                if (target2 == 3)
                    x = beaconPos1[0];
                if (target2 == 1)
                    x = beaconPos2[0];
                if (realign(heading)) {
                    if (isVisible) {
                        if (line.alpha() > 20)
                            sideOfLine = 0;//on target
                        else if (posx < x)
                            sideOfLine = -1;//left of target
                        else if (posx > x)
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
                if (sonar.cmUltrasonic() < 10) {
                    allStop();
                    control = RobotSteps.BACK_UP;
                    break;
                }
                int x = 0;
                if (target2 == 3)
                    x = beaconPos1[0];
                if (target2 == 1)
                    x = beaconPos2[0];
                if (sideOfLine == 0) {
                    control = RobotSteps.PRESCAN;
                    segmentTime = time;
                } /*else if (spareSonar.cmUltrasonic() < 45 || segmentTime + 1000 < time) {
                    navigateBlind(0, .3, heading);
                    if (spareSonar.cmUltrasonic() < 45)
                        segmentTime = time;
                } else {
                    if (isVisible)
                        allStop();
                    if (sideOfLine == -1) {//left
                        navigateBlind(180, .35, heading);
                        if (posx - 20 > x)
                            sideOfLine = 1;
                        if (line.alpha() > 20)
                            sideOfLine = 0;
                    }
                    if (sideOfLine == 1) {//right
                        navigateBlind(0, .35, heading);
                        if (posx + 20 < x)
                            sideOfLine = -1;
                        if (line.alpha() > 20)
                            sideOfLine = 0;
                    }
                }
                telemetry.addData("Status", "Finding line...");
                break;
            }
            case BACK_UP: {
                allStop();
                navigateBlind(270, .4, heading);
                if (sonar.cmUltrasonic() > 13) {
                    control = RobotSteps.REALIGN;
                }
                break;
            }*/
            case PRESCAN: {
                if (sonar.cmUltrasonic() > 16) {
                    navigateBlind(90, .5, heading);
                    allStop();
                } else if (sonar.cmUltrasonic() < 12) {
                    navigateBlind(270, .5, heading);
                    allStop();
                } else {
                    control = RobotSteps.INIT_SCAN;
                }
            }
            case INIT_SCAN: {
                allStop();
                if (realign(heading)) {
                    control = RobotSteps.SCAN;
                }
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
                    if (sonar.cmUltrasonic() > 20)
                        pushCheck = 1;
                    if (sonar.cmUltrasonic() < 15)
                        pushCheck = -1;
                }
                break;
            }
            case CHECK_PUSH: {
                if (sonar.cmUltrasonic() > 15 && sonar.cmUltrasonic() < 20)
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
                if (segmentTime + 750 < time)
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
                if (sonar.cmUltrasonic() > 30 && sonar.cmUltrasonic() != 255) {
                    control = RobotSteps.REREALIGN;
                    if (target2 == 1) {
                        control = RobotSteps.MOVE_TO_PARK;
                        segmentTime = time;
                    }
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
                pusher.setPosition(.25);
                scan(allTrackables.get(target2));
                if (segmentTime < time) {
                    control = RobotSteps.MOVE_TO_BEACON2;
                    segmentTime = time;
                }
                break;
            }
            case MOVE_TO_BEACON2: {
                boolean isVisible = scan(allTrackables.get(target2));
                spareSonarCache = spareSonarReader.read(0x04, 1);

                int moveAngle;
                if (sonar.cmUltrasonic() < 40)
                    moveAngle = 180;
                else
                    moveAngle = 170;
                if (segmentTime + 1000 > time)
                    navigateBlind(moveAngle, .7, heading);
                else
                    navigateBlind(moveAngle, .4, heading);

                if ((isVisible && posx > 1170) || (line.alpha() > 20 && segmentTime + 800 < time) || ((spareSonarCache[0] & 0xff) <= 65 && segmentTime + 800 < time)) {
                    control = RobotSteps.B2_LINEUP_CHECK;
                    allStop();
                }
                break;
            }
            case B2_LINEUP_CHECK: {
                allStop();
                navigateBlind(90, .4, heading);
                if (sonar.cmUltrasonic() < 35) {
                    control = RobotSteps.INIT_ALIGN;
                    segmentTime = time;
                    allStop();
                }
                break;
            }
            case MOVE_TO_PARK: {
                double pow;
                if (segmentTime + 1250 > time)
                    pow = .5;
                else
                    pow = .7;
                if (navigateTime(300, pow, 2000, heading)) {
                    allStop();
                    control = RobotSteps.COMPLETE;
                }
                break;
            }
            default: {//Hopefully this only runs when program ends
                allStop();
                int sr = sweeper.getCurrentPosition() % 1440;
                if (sr < 0) {
                    sr = 1440 - Math.abs(sr);
                }
                if (sr <= 50 || sr >= 1390) {
                    sweeper.setPower(0);
                } else if (sr <= 360) {
                    sweeper.setPower(-.1);
                } else if (sr > 360) {
                    sweeper.setPower(.15);
                }
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