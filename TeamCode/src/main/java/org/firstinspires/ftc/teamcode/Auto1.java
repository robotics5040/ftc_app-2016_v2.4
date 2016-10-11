package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
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
@Autonomous (name = "Red 1", group = "Red Autonomous")
public class Auto1 extends OpMode {
    int control = 2, degrees = 0, previousHeading = 0, heading, trueHeading, target, startDegrees;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    GyroSensor gyro;
    float robotBearing;
    Long time, startTime, segmentTime;
    VuforiaLocalizer vuforia;
    List<VuforiaTrackable> allTrackables;
    double posx, posy, startx, starty;
    float mmFTCFieldWidth;

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;
    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        gyro = hardwareMap.gyroSensor.get("gyro");
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
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables targets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        VuforiaTrackable redTools  = targets.get(1); //load tools
        redTools.setName("Tools");

        VuforiaTrackable redGears  = targets.get(3); //load gears
        redGears.setName("Gears");

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = (float)16.5 * mmPerInch;            // ... or whatever is right for your robot
        mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

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
                .translation(mmBotWidth/2,(float)44.45,200)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)redTools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)redGears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        targets.activate();
    }

    public void loop()
    {
        for (VuforiaTrackable trackable : allTrackables) {
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }

        time = System.currentTimeMillis();

        heading = gyro.getHeading();
        trueHeading = degrees + heading;
        checkHeading();

        switch (control)
        {
            case 0: {//Set up for initial delay
                segmentTime = time;
                control = 1;
            }
            case 1: {//Initial delay, set control to 2 to skip delay
                if (segmentTime + 10000 < time)
                    control = 2;
                break;
            }
            case 2: {//setup statement

            }
            case 3: {//move into position to shoot (timed move)

            }
            case 4: {//start shooting

            }
            case 5: {//delay while shooting

            }
            case 6: {//setup for turn

            }
            case 7: {//turn 90 degrees left

            }
            case 8: {//setup for move

            }
            case 9: {//move until target is visible

            }
            case 10: {//setup for lineup

            }
            case 11: {//attempt to lineup

            }
            case 12: {//check lineup, return to 10 if failed

            }
            case 13: {//move to beacon

            }
            case 14: {//check beacon

            }
            case 15: {//press a button

            }
            default: {
                allStop();
            }
        }

        telemetry.addData("Timer", time - startTime);
        telemetry.addData("Control", control);
        telemetry.addData("Heading", trueHeading);

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

    public boolean shoot() //Waiting for launcher to be built, no code implemented
    {
        return true;
    }
}
