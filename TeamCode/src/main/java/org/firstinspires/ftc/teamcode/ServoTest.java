package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by bense on 9/26/2016.
 */

public class ServoTest extends OpMode {

    Servo servo;

    public void init()
    {
        servo = hardwareMap.servo.get("servo");
    }

    public void loop()
    {
        if (gamepad1.left_bumper)
            servo.setPosition(1);
        if (gamepad1.right_bumper)
            servo.setPosition(0);
    }
}
