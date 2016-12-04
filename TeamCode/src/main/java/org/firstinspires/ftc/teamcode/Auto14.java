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
 * Created by bense on 11/11/2016.
 */
@Autonomous(name = "Blue Pos 1: Shoot 2/Return to Start", group = "Blue Autonomous")
public class Auto14 extends OpMode {
    int target, startDegrees, targetDegrees, shooterStartPos;
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
    boolean lineUsed = false;

    public static final String TAG = "Vuforia Sample";

    public enum RobotSteps {INIT_START, DELAY, INIT_MOVE, MOVE_TO_SHOOT, INIT_SHOOT, SHOOT, RETURN, PARK, ALL_DONE, SWEEPER_MOVE_BACKWARD, SWEEPER_MOVE_FORWARD, SHOOT_DOS};
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
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
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
            case INIT_START: {//Set up for initial delay
                segmentTime = time;
                control = RobotSteps.DELAY;
                telemetry.addData("Status", "Setting up start delay...");
                break;
            }
            case DELAY: {//Initial delay, set control to 2 to skip delay
                if (segmentTime + 5000 < time) //set to 3 seconds for testing
                    control = RobotSteps.INIT_MOVE;
                telemetry.addData("Status", "Waiting to start...");
                break;
            }
            case INIT_MOVE: {//setup for first move
                startDegrees = heading;
                segmentTime = time;
                control = RobotSteps.MOVE_TO_SHOOT;
                telemetry.addData("Status", "Preparing to move...");
                break;
            }
            case MOVE_TO_SHOOT: {//move into position to shoot (timed move)
                if (navigateTime(180, .6, 850, heading))
                    control = RobotSteps.INIT_SHOOT;
                telemetry.addData("Status", "Moving for 1 seconds...");
                break;
            }
            case INIT_SHOOT: {//Setup for shoot
                allStop();
                control = RobotSteps.SHOOT;
                shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//changed ruhn using encoder to run to position
                //shooterStartPos = shooter.getCurrentPosition();
                shooterStartPos = 0;
                shooter.setTargetPosition(-1340);
                break;
            }
            case SHOOT: {//shoot
                if (!shoot() ) {
                    control = RobotSteps.SWEEPER_MOVE_BACKWARD;
                    segmentTime = time;
                }
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
                    control = RobotSteps.SHOOT_DOS;
                    shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    sweeper.setPower(0);
                    shooter.setTargetPosition(-1340);
                }
                break;

            }
            case SHOOT_DOS: {//shoot^2
                if (!shoot() ) {
                    control = RobotSteps.RETURN;
                    segmentTime = time;
                }
                break;

            }
            case RETURN: {//move forward to knock off cap ball
                if (navigateTime(0, .5, 850, heading))
                    control = RobotSteps.ALL_DONE;
                break;
            }
            /*case PARK: {//park on center
                if (navigateTime(270, .5, 2500, heading))
                    control = RobotSteps.ALL_DONE;
                break;
            }*/
            default: {//Hopefully this only runs when program ends
                allStop();
                telemetry.addData("Status", "Switch is in default. Waiting for autonomous to end...");
            }
        }

        telemetry.addData("Timer", time - segmentTime);
        telemetry.addData("Control", control);
        telemetry.addData("Heading", heading);
        telemetry.addData("Sonar", sonar.getUltrasonicLevel());
        telemetry.addData("Sweeper Power",sweeper.getPower());
        telemetry.addData("Sweeper Position",sweeper.getCurrentPosition());

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
            return .1;
        if (h < startDegrees - 10)
            return -.1;
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

    public void scan(VuforiaTrackable t) //for t, use allTrackables.get(). 0 is Wheels, 1 is Tools, 2 is Legos, 3 is Gears
    {
        telemetry.addData(t.getName(), ((VuforiaTrackableDefaultListener) t.getListener()).isVisible() ? "Visible" : "Not Visible");    //

        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) t.getListener()).getUpdatedRobotLocation();
        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;
        }
    }
}
