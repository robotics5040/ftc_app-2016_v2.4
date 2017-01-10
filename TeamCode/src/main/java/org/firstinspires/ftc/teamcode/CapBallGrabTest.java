package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by bense on 1/8/2017.
 */
@TeleOp (name = "Grabber", group = "Testing")
public class CapBallGrabTest extends OpMode {
    Servo armLeft;
    Servo armRight;
    DcMotor lift;

    boolean dpadPressed = false;
    int count = 32;

    final int MAX_COUNT = 32, MIN_COUNT = 30;

    public void init() {
        armLeft = hardwareMap.servo.get("armLeft");
        armRight = hardwareMap.servo.get("armRight");
        lift = hardwareMap.dcMotor.get("lift");
    }

    public void loop() {
        if (!gamepad1.dpad_down && !gamepad1.dpad_up)
            dpadPressed = false;
        if ((gamepad1.dpad_down || gamepad1.dpad_up) && !dpadPressed) {
            dpadPressed = true;
            if (gamepad1.dpad_down)
                count--;
            if (gamepad1.dpad_up)
                count++;
            if (count > MAX_COUNT)
                count--;
            if (count < MIN_COUNT)
                count++;
        }
        armLeft.setPosition(((double)count / MAX_COUNT));
        armRight.setPosition((double)(MAX_COUNT - count) / MAX_COUNT);
        if (gamepad1.left_bumper)
            count = MAX_COUNT;
        if (gamepad1.right_bumper)
            count = MIN_COUNT;

        lift.setPower(-gamepad1.right_stick_y);

        telemetry.addData("Count", count);
        telemetry.addData("Max count", MAX_COUNT);
        telemetry.addData("Servo pos", "Left: " + ((double)count / MAX_COUNT) + "   Right: " + (double)(MAX_COUNT - count)/MAX_COUNT);
        telemetry.addData("Servo pos (ACTUAL)", "Left: " + armLeft.getPosition() + "   Right: " + armRight.getPosition());
    }
}
