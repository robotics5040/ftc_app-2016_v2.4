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

    public void loop()
    {
        double rightStickX = gamepad1.right_stick_x;
        double rightStickY = gamepad1.right_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;
        if (gamepad1.right_bumper)
        {
            rightStickX *= .5;
            rightStickY *= -.5;
            leftStickX *= .5;
            leftStickY *= .5;
        }
        else {
            rightStickX *= .75;
            rightStickY *= -.75;
            leftStickX *= .75;
            leftStickY *= .75;
        }//                  direction                               rotation
        frontLeft.setPower(((-rightStickY - rightStickX)/2) * .75 + (-.25 * leftStickX));
        backLeft.setPower(((-rightStickY + rightStickX)/2) * .75 + (-.25 * leftStickX));
        frontRight.setPower(((rightStickY - rightStickX)/2) * .75 + (-.25 * leftStickX));
        backRight.setPower(((rightStickY + rightStickX)/2) * .75 + (-.25 * leftStickX));

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
