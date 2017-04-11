package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by bense on 2/27/2017.
 */
@Autonomous (name = "Servo Init", group = "Testing")
public class ServoInit extends OpMode {
    Servo s1, s2;

    public void init() {
        s1 = hardwareMap.servo.get("pusher");
        s2 = hardwareMap.servo.get("pusher2");

        s1.setPosition(1);
        s2.setPosition(0);
    }

    public void loop() {
        s1.setPosition(0);
        s2.setPosition(1);
    }
}
