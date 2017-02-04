package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by bense on 11/1/2016.
 */
@TeleOp(name = "CompOmnibot4", group = "Competition")
public class CompetitionOmnibot4 extends OpMode {
    //motor variables
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor sweeper;
    DcMotor shooter;
    String loopNumber;
    //program variables
    int controlMode = 1, sweep = 0, shooterResetPos, sweeperResetPos, shootCount = 0;
    boolean shoot = false, reset = false, motorReset = false, aPressed = true, manualRest = false, sweepResetCheck = true;
    //gyro thingies
    long segmentTime;
    GyroSensor gyro;
    int previousHeading = 0, heading = 0, degrees = 0, trueHeading = 0;

    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        shooter = hardwareMap.dcMotor.get("shooter");
        gyro = hardwareMap.gyroSensor.get("gyro");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        sweeper.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        sweeper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sweeperResetPos = sweeper.getCurrentPosition();
        shooterResetPos = shooter.getCurrentPosition();
        shooter.setTargetPosition(0);
    }

    public void loop() {
        /*heading = gyro.getHeading();
        trueHeading = degrees + heading;
        checkHeading();*/
        heading = gyro.getHeading();
        if (heading > 180)
            heading -= 360;

        //gamepad input
        double rx = gamepad1.right_stick_x;
        double ry = gamepad1.right_stick_y;
        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        if (controlMode == 2) {
            double ph = rx;
            rx = -ry;
            ry = ph;
        }
        if (controlMode == 0)
        {
            double ph = rx;
            rx = ry;
            ry = -ph;
        }
        if (controlMode == 3)
        {
            rx *= -1;
            ry *= -1;
        }

        if (gamepad1.dpad_up)
            controlMode = 3;
        if (gamepad1.dpad_right)
            controlMode = 0;
        if (gamepad1.dpad_down)
            controlMode = 1;
        if (gamepad1.dpad_left)
            controlMode = 2;

        //power multipliers - so that gamepad controls do not function at 100%
        if (!gamepad1.right_bumper)
        {
            rx *= .75;
            ry *= -.75;
            lx *= .75;
            ly *= .75;
        }
        else
        {
            ry *= -1;
        }
        //                  direction                               rotation
        //average of the joystick inputs + rotation
        frontLeft.setPower(((-ry - rx)/2) * .75 + (-.25 * lx));
        backLeft.setPower(((-ry + rx)/2) * .75 + (-.25 * lx));
        frontRight.setPower(((ry - rx)/2) * .75 + (-.25 * lx));
        backRight.setPower(((ry + rx)/2) * .75 + (-.25 * lx));

        if (gamepad2.dpad_down && !manualRest) {
            sweeper.setPower(-1);
            sweep = -1;
        }
        if ((gamepad2.dpad_left || gamepad2.dpad_right) && !manualRest) {
            sweep = 0;
            sweepResetCheck = false;
        }
        if (gamepad2.dpad_up && !manualRest) {
            sweeper.setPower(1);
            sweep = 1;
        }
        //if (!gamepad2.a)//stops program from looping more than once, on shot per one button press
        //    aPressed = true;
        if (gamepad2.a && sweep == 0 && aPressed && !shoot && !manualRest) {
            shooter.setTargetPosition(-1200 + shootCount*-1440);
            shootCount++;

            shoot = true;
            aPressed = false;
            //sets button pressed to true, sets to false after ball has been shot
        }
        if (shoot) {
            if (!shoot()) {
                shoot = false;
                shooter.setPower(0);
            }
        }





        telemetry.addData("Shooter pos", shooter.getCurrentPosition());
        telemetry.addData("Sweeper pos", sweeper.getCurrentPosition());
        telemetry.addData("Sweeper reset threshold", sweeper.getCurrentPosition() % 720);
        telemetry.addData("Sweeper State", sweep);
        telemetry.addData("Shooter Target Pos", shooter.getTargetPosition());
        telemetry.addData("", "");
        telemetry.addData("Gyro heading", heading);
        telemetry.addData("Rotations from start", degrees / 360);
        telemetry.addData("True Heading", trueHeading);
        telemetry.addData("raw x", gyro.rawX());
        telemetry.addData("raw y", gyro.rawY());
        telemetry.addData("raw z", gyro.rawZ());
        telemetry.addData("Shoot T/F", shoot);
        telemetry.addData("Reset T/F", reset);
        telemetry.addData("Shooter Power", shooter.getPower());
        telemetry.addData("Loop", "No loop");
        telemetry.addData("Busy T/F",shooter.isBusy());
        telemetry.addData("A pressed", gamepad2.a);
        telemetry.addData("Manual Reset", manualRest);
        telemetry.addData("Output of right Stick", gamepad2.right_stick_y);
        telemetry.addData("Output of left bumper", gamepad2.left_bumper);
        telemetry.addData("Loop Number", loopNumber);
        if (sweep == 0 && !manualRest/* && !sweepResetCheck*/) {
            int sr = sweeper.getCurrentPosition() % 720;
            if (sr < 0)
                sr = 720 - Math.abs(sr);
            if (sr <= 100 || sr >= 620) {
                sweeper.setPower(0);
                sweepResetCheck = true;
            } else if (sr <= 360) {
                sweeper.setPower(-.08);
            } else if (sr > 360) {
                sweeper.setPower(.15);
            }
        }
        else if (sweep == 0 && !manualRest)
            sweeper.setPower(0);

        //Manual fire
        if (gamepad2.left_bumper)
        {
            manualRest = true;
            if (gamepad2.right_bumper) {
                shooter.setPower(.5);
            } else {
                shooter.setPower(0);
            }

            if (gamepad2.dpad_up)
                sweeper.setPower(.15);
            else
                sweeper.setPower(0);

        } else if (manualRest) {
            shooter.setPower(0);
            manualRest = false;
            shootCount = 0;
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterResetPos = 0;
            shooter.setTargetPosition(0);
            sweeper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sweep = 0;
        }

        if (motorReset) {
            motorReset = false;
            shooter.setPower(0);
            //shooter.setPower(shooter.getPower()-shooter.getPower());
        }
        //previousHeading = gyro.getHeading();
    }

    public void checkHeading()
    {
        if (previousHeading - heading > 270)
            degrees += 360;
        else if (heading - previousHeading > 270)
            degrees -= 360;
    }

    public boolean shoot() //Autonomous code modified for teleop
    {
        boolean returnStatement = false;

        if (shooter.getCurrentPosition() > shooter.getTargetPosition()) {
            shooter.setPower(.2);
            loopNumber = "If statement";
            returnStatement = true;
        }
        else if (shooter.getCurrentPosition() <= shooter.getTargetPosition()) {
            shooter.setPower(0);
            aPressed = true;
            loopNumber = "Else if statement ";
            returnStatement = false;
        }
        return returnStatement;
    }
}
//this is an easter egg