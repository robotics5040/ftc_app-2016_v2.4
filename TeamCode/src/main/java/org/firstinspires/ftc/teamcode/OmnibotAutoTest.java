package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
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
 * Created by bense on 9/19/2016.
 */
//change
@Autonomous(name = "Sensor Read", group = "Testing")
public class OmnibotAutoTest extends OpMode {
    int control = 0, degrees = 0, previousHeading = 0, heading, trueHeading, target, start, startDegrees, targetDegrees, rotateDegrees = 0;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor shooter;
    DcMotor sweeper;
    GyroSensor gyro;
    float robotBearing;
    Long time, startTime, startTime2;
    VuforiaLocalizer vuforia;
    List<VuforiaTrackable> allTrackables;
    double posx, posy, posz, startx, starty;
    float mmFTCFieldWidth;
    ColorSensor color, line, lineLeft, lineRight, color2;
    ModernRoboticsI2cRangeSensor sonar;
    I2cDevice sonar2;
    I2cDevice wallSonar;

    I2cDeviceSynch sonar2Reader;
    I2cDeviceSynch wallSonarReader;

    byte[] sonar2Cache;
    byte[] wallSonarCache;

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;

    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        shooter = hardwareMap.dcMotor.get("shooter");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        color = hardwareMap.colorSensor.get("color");

        color2 = hardwareMap.colorSensor.get("color2");
        color2.setI2cAddress(I2cAddr.create8bit(0x3a));

        line = hardwareMap.colorSensor.get("line");
        line.setI2cAddress(I2cAddr.create8bit(0x42));

        //I2cAddr lineAddressLeft = new I2cAddr(0x20);
        lineLeft = hardwareMap.colorSensor.get("lineLeft");
        lineLeft.setI2cAddress(I2cAddr.create8bit(0x40));

        //I2cAddr lineAddressRight = new I2cAddr(0x21);
        lineRight = hardwareMap.colorSensor.get("lineRight");
        lineRight.setI2cAddress(I2cAddr.create8bit(0x3e));

        color.enableLed(false);
        line.enableLed(true);
        lineLeft.enableLed(true);
        lineRight.enableLed(true);
        sonar = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sonar");
        sonar.setI2cAddress(I2cAddr.create8bit(0x28));

        sonar2 = hardwareMap.i2cDevice.get("sonar2");
        wallSonar = hardwareMap.i2cDevice.get("wallSonar");

        sonar2Reader = new I2cDeviceSynchImpl(sonar2, I2cAddr.create8bit(0x2c), false);
        wallSonarReader = new I2cDeviceSynchImpl(wallSonar, I2cAddr.create8bit(0x2a), false);

        sonar2Reader.engage();
        wallSonarReader.engage();

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        startTime = System.currentTimeMillis();
        time = startTime;
        startTime2 = startTime;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AUBrQCz/////AAAAGXg5njs2FEpBgEGX/o6QppZq8c+tG+wbAB+cjpPcC5bwtGmv+kD1lqGbNrlHctdvrdmTJ9Fm1OseZYM15VBaiF++ICnjCSY/IHPhjGW9TXDMAOv/Pdz/T5H86PduPVVKvdGiQ/gpE8v6HePezWRRWG6CTA21itPZfj0xDuHdqrAGGiIQXcUbCTfRAkY7HwwRfQOM1aDhmeAaOvkPPCnaA228iposAByBHmA2rkx4/SmTtN82rtOoRn3/I1PA9RxMiWHWlU67yMQW4ExpTe2eRtq7fPGCCjFeXqOl57au/rZySASURemt7pwbprumwoyqYLgK9eJ6hC2UqkJO5GFzTi3XiDNOYcaFOkP71P5NE/BB";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables targets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable blueWheels = targets.get(0);
        blueWheels.setName("Wheels");

        VuforiaTrackable redTools  = targets.get(1);
        redTools.setName("Tools");

        VuforiaTrackable blueLegos = targets.get(2);
        blueLegos.setName("Legos");

        VuforiaTrackable redGears  = targets.get(3);
        redGears.setName("Gears");

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = (float)16.5 * mmPerInch;            // ... or whatever is right for your robot
        mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        OpenGLMatrix blueWheelsLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the RED WALL. Our translation here
                    is a negative translation in X.*/
                .translation(mmFTCFieldWidth/2, mmFTCFieldWidth/2 - (float)2082.8, 0)
                .multiplied(Orientation.getRotationMatrix(
                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        blueWheels.setLocation(blueWheelsLocationOnField);
        RobotLog.ii(TAG, "Wheels=%s", format(blueWheelsLocationOnField));

        OpenGLMatrix redToolsLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(mmFTCFieldWidth/2 - (float)863.6, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        redTools.setLocation(redToolsLocationOnField);
        RobotLog.ii(TAG, "Tools=%s", format(redToolsLocationOnField));

        OpenGLMatrix blueLegosLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the RED WALL. Our translation here
                    is a negative translation in X.*/
                .translation(mmFTCFieldWidth/2, mmFTCFieldWidth/2 - (float)863.6, 0)
                .multiplied(Orientation.getRotationMatrix(
                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        blueLegos.setLocation(blueLegosLocationOnField);
        RobotLog.ii(TAG, "Legos=%s", format(blueLegosLocationOnField));

        OpenGLMatrix redGearsLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth/2 - (float)2082.8, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        redGears.setLocation(redGearsLocationOnField);
        RobotLog.ii(TAG, "Gears=%s", format(redGearsLocationOnField));

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,(float)44.45 + 175,200)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX, //Changed from YZY
                        AngleUnit.DEGREES, -90, -90, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)blueWheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)redTools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueLegos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)redGears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        targets.activate();
    }

    public void loop() {
        for (VuforiaTrackable trackable : allTrackables) {
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }

        if (gamepad1.dpad_up) {
            sonar2Cache = sonar2Reader.read(0x04, 1);
            wallSonarCache = wallSonarReader.read(0x04, 1);

            telemetry.addData("Sonar", sonar.cmUltrasonic());
            telemetry.addData("Sonar 2", sonar2Cache[0] & 0xff);
            telemetry.addData("Wall Sonar", wallSonarCache[0] & 0xff);
        }
        else if (gamepad1.dpad_down) {
            telemetry.addData("Line Alpha", line.alpha());
            telemetry.addData("Line Left", lineLeft.alpha());
            telemetry.addData("Line Right", lineRight.alpha());
        }
        else if (gamepad1.dpad_left) {
            telemetry.addData("Red 2", color2.red());
            telemetry.addData("Blue 2", color2.blue());
            telemetry.addData("Red 1", color.red());
            telemetry.addData("Blue 1", color.blue());
        }
        else if (gamepad1.dpad_right) {
            int heading = gyro.getHeading() - rotateDegrees;
            if (heading + rotateDegrees > 270)
                heading -= 360;

            telemetry.addData("Gyro", heading);
            telemetry.addData("Actual Rotation", trueHeading);
            telemetry.addData("Target", target);
        }
        else if (gamepad1.a) {
            telemetry.addData("Sweeper", sweeper.getCurrentPosition());
            telemetry.addData("Shooter", shooter.getCurrentPosition());
            if (gamepad1.b) {
                sweeper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        if (lastLocation != null) {
            VectorF trans = lastLocation.getTranslation();
            Orientation rot = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
            posx = trans.get(0);
            posy = trans.get(1);

            robotBearing = rot.thirdAngle;

            //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
            String outputData = format(lastLocation);

            /*outputData = outputData.substring(outputData.lastIndexOf("{") + 1, outputData.lastIndexOf("}"));
            posx = Double.parseDouble(outputData.substring(0, outputData.indexOf(" ")));
            outputData = outputData.substring(outputData.indexOf(" ") + 1);
            posy = Double.parseDouble(outputData.substring(0, outputData.indexOf(" ")));
            outputData = outputData.substring(outputData.indexOf(" ") + 1);
            posz = Double.parseDouble(outputData);*/
            telemetry.addData("posx", posx);
            telemetry.addData("posy", posy);
            //telemetry.addData("posz", posz);

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
