package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by bense on 11/1/2016.
 */
@TeleOp(name = "CompOmnibot2", group = "Competition")
public class CompetitionOmnibot2 extends OpMode {
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
    boolean shoot = false, reset = false, motorReset = false, aPressed = true, manualRest = false;
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

        sweeper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        if (gamepad1.right_bumper)
        {
            rx *= .5;
            ry *= -.5;
            lx *= .5;
            ly *= .5;
        }
        else {
            rx *= .75;
            ry *= -.75;
            lx *= .75;
            ly *= .75;
        }
        //                  direction                               rotation
        //average of the joystick inputs + rotation
        frontLeft.setPower(((-ry - rx)/2) * .75 + (-.25 * lx));
        backLeft.setPower(((-ry + rx)/2) * .75 + (-.25 * lx));
        frontRight.setPower(((ry - rx)/2) * .75 + (-.25 * lx));
        backRight.setPower(((ry + rx)/2) * .75 + (-.25 * lx));

        if (gamepad2.dpad_down) {
            sweeper.setPower(-1);
            sweep = -1;
        }
        if (gamepad2.dpad_left || gamepad2.dpad_right) {
            sweep = 0;
        }
        if (gamepad2.dpad_up) {
            sweeper.setPower(1);
            sweep = 1;
        }
        //if (!gamepad2.a)//stops program from looping more than once, on shot per one button press
        //    aPressed = true;
        if (gamepad2.a && sweep == 0 && aPressed && shoot == false) {
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
        if (sweeper.getCurrentPosition() % 270 >= 90 + sweeperResetPos && sweep == 0)
            sweeper.setPower(.15);
        else if (sweep == 0)
            sweeper.setPower(0);


        //Shooting + automatic reset
        /*if (shoot == true)
        {
            if (shooter.getCurrentPosition() > shooter.getTargetPosition() ) {
                telemetry.addData("Loop", "Shooting");
                if (segmentTime + 200 < System.currentTimeMillis()) {
                    shooter.setPower(.5);
                }else
                    shooter.setPower(1);
            }
            else {
                if (shooter.getCurrentPosition() <= shooter.getTargetPosition()&& !shoot) {
                    telemetry.addData("Loop", "Reseting");
                    shooter.setPower(-.1);
                    if (reset)
                        motorReset = true;
                    reset = false;
                } else {
                    telemetry.addData("Loop", "Reset");
                    shooter.setPower(0);
                    shoot = false;
                }
            }
        }*/
        //Manual fire
        if (gamepad2.left_bumper)
        {
            //
            if ((gamepad2.right_stick_y < -.1 || gamepad2.right_stick_y > .1) && gamepad2.left_bumper) {
                //shooter.setPower(gamepad2.right_stick_y);
                shooter.setPower(.6);
                manualRest = true;
            }

//                if (gamepad2.y)
//            {
//                shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                shooterResetPos = 0;
//                shooter.setTargetPosition(0);
//            }
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
//test test test