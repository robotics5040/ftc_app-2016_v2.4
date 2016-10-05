package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by bense on 9/14/2016.
 */
@TeleOp(name = "Motor Tet", group = "Tests")
public class MotorTest extends OpMode {
    DcMotor motor1;
    DcMotor motor2;
    public void init()
    {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
    }

    public void loop()
    {
        motor1.setPower(gamepad1.right_stick_y);
        motor2.setPower(gamepad1.left_stick_y);
    }
}
