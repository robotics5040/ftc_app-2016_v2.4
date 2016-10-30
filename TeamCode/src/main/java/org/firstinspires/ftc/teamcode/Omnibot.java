package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by bense on 8/31/2016.
 */
@TeleOp(name = "Omnibot", group = "Main")
public class Omnibot extends OpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    int controlMode = 1;

    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop() {
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
            controlMode = 0;
        if (gamepad1.dpad_right)
            controlMode = 1;
        if (gamepad1.dpad_down)
            controlMode = 2;
        if (gamepad1.dpad_left)
            controlMode = 3;

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
        }//                  direction                               rotation
        //average of the joystick inputs + rotation
        frontLeft.setPower(((-ry - rx)/2) * .75 + (-.25 * lx));
        backLeft.setPower(((-ry + rx)/2) * .75 + (-.25 * lx));
        frontRight.setPower(((ry - rx)/2) * .75 + (-.25 * lx));
        backRight.setPower(((ry + rx)/2) * .75 + (-.25 * lx));

        if (gamepad1.x)
            frontLeft.setPower(.5);
        if (gamepad1.a)
            backLeft.setPower(.5);
        if (gamepad1.y)
            frontRight.setPower(.5);
        if (gamepad1.b)
            backRight.setPower(.5);
    }
}
