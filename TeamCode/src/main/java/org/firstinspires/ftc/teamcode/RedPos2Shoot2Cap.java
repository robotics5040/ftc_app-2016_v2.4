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
@Autonomous(name = "Red Pos 2: Shoot 2/Hit cap ball/Park on center :)", group = "Red Autonomous2")
public class RedPos2Shoot2Cap extends OpMode {
    int target, startDegrees, targetDegrees, shooterStartPos, rotateDegrees = 0;
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
    boolean lineUsed = false;

    public static final String TAG = "Vuforia Sample";

    public enum RobotSteps {INIT_START, DELAY, INIT_MOVE, MOVE_TO_SHOOT, INIT_SHOOT, SHOOT, RETURN, PARK, ALL_DONE,
        SWEEPER_MOVE_BACKWARD, SWEEPER_MOVE_FORWARD, SHOOT_DOS,SPIN2, ALIGN, DELAY2, DELAY_FOR_SHOOT, STRAFE, TURN};
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

        gyro.calibrate();

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sweeper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        startTime = System.currentTimeMillis();
        time = startTime;
        segmentTime = startTime;
    }

    public void loop()
    {
        time = System.currentTimeMillis();

        int heading = gyro.getHeading() - rotateDegrees;
        if (heading + rotateDegrees > 270)
            heading -= 360;

        switch (control)
        {
            case INIT_START: {//Set up for initial delay
                segmentTime = time;
                control = RobotSteps.DELAY;
                telemetry.addData("Status", "Setting up start delay...");
                break;
            }
            case DELAY: {//Initial delay
                if (segmentTime + 15000 < time)
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
                if (navigateTime(180, .65, 1500, heading)) {
                    control = RobotSteps.DELAY_FOR_SHOOT;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Moving for 1.5 seconds...");
                break;
            }
            case DELAY_FOR_SHOOT: {
                allStop();
                if (segmentTime + 500 < time)
                    control = RobotSteps.INIT_SHOOT;
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
                sweeper.setPower(.5);
                if (segmentTime + 2300 < time)
                {
                    control = RobotSteps.SHOOT_DOS;
                    shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    sweeper.setPower(0);
                    shooter.setTargetPosition(-1340);
                    segmentTime = time;

                }

                break;

            }
            case SHOOT_DOS: {//shoot^2
                int sr = sweeper.getCurrentPosition() % 720;
                if (!shoot() ) {
                    control = RobotSteps.RETURN;
                    segmentTime = time;
                }

                if (sr < 0) {
                    sr = 720 - Math.abs(sr);
                }
                if (sr <= 100 || sr >= 620) {
                    sweeper.setPower(0);
                } else if (sr <= 360) {
                    sweeper.setPower(-.08);
                } else if (sr > 360) {
                    sweeper.setPower(.15);
                }
                break;
            }

            case RETURN: {//move forward to knock off cap ball
                if (navigateTime(180, .6, 1500, heading)) {
                    control = RobotSteps.TURN;
                    segmentTime = time;
                }
                telemetry.addData("Status", "Made it to the delay case");
                break;
            }
            case TURN: {
                rotateDegrees = -45;
                if (realign(heading)) {
                    segmentTime = time;
                    control = RobotSteps.STRAFE;
                }
                break;
            }
            case STRAFE: {
                if (navigateTime(270, .7, 750, heading))
                    control = RobotSteps.DELAY2;
                break;
            }
            /*case DELAY2: {//Initial delay, set control to 2 to skip delay
                if (segmentTime + 500 < time) //set to 3 seconds for testing
                    control = RobotSteps.PARK;
                telemetry.addData("Status", "Waiting to start...");
                break;
            }*/
            /*case SPIN2: {//Turn
                if (rotate('l', 180, heading)) {
                    control = RobotSteps.ALIGN;
                    rotateDegrees = 180;
                    allStop();
                }
                telemetry.addData("Status", "Turning around...");
                break;
            }
            case ALIGN:{
                if(realign(heading)) {
                    allStop();
                    control = RobotSteps.PARK;
                    segmentTime = time;
                }
                break;
            }

            case PARK: {//park
                if (navigateTime(180, .5, 1000, heading))
                    control = RobotSteps.ALL_DONE;
                break;
            }
            */
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

        telemetry.addData("Timer", time - segmentTime);
        telemetry.addData("Control", control);
        telemetry.addData("Heading", heading);
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
        return (h * .01)/2;
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
    public boolean realign (int h)
    {
        if (h + 4 < startDegrees) {
            frontRight.setPower(-.08);
            frontLeft.setPower(-.08);
            backRight.setPower(-.08);
            backLeft.setPower(-.08);
            segmentTime = time;
        } else if (h - 4 > startDegrees) {
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
