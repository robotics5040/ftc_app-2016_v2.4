package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by bense on 9/26/2016.
 */

@TeleOp(name = "ServoTest", group = "Testing")
public class ServoTest extends OpMode {

    Servo servo;
    int count = 0, iteration = 45;
    boolean aPressed = false;

    public void init()
    {
        servo = hardwareMap.servo.get("servo");
        servo.setPosition(0);
    }

    public void loop()
    {
        if (!gamepad1.dpad_down && !gamepad1.dpad_up)
            aPressed = false;
        if ((gamepad1.dpad_down || gamepad1.dpad_up) && !aPressed) {
            aPressed = true;
            if (gamepad1.dpad_down)
                count--;
            if (gamepad1.dpad_up)
                count++;
            if (count * iteration > 360)
                count--;
            if (count * iteration < 0)
                count++;
            servo.setPosition(((double)count * iteration)/360);
        }
        telemetry.addData("Degrees", count * iteration);
        telemetry.addData("A pressed", aPressed);
    }
}
