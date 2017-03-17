package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by bense on 3/16/2017.
 */
@TeleOp (name = "Sweeper Reset Test", group = "Testing")
public class ResetTesting extends OpMode {
    int sweep = 0, testTime = 0;
    boolean manualRest = false, sweepResetCheck = false, segmentRunning = false;
    long segmentTime, startTime = 0, time;

    DcMotor sweeper;
    public void init() {
        sweeper = hardwareMap.dcMotor.get("sweeper");
        sweeper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop() {
        time = System.currentTimeMillis();
        if (startTime == 0) {
            startTime = time;
            segmentTime = time;
        }

        if (gamepad1.x && !segmentRunning) {
            segmentTime = time;
            testTime = 1000;
            segmentRunning = true;
        }
        if (gamepad1.y && !segmentRunning) {
            segmentTime = time;
            testTime = 3000;
            segmentRunning = true;
        }
        if (gamepad1.b && !segmentRunning) {
            segmentTime = time;
            testTime = 5000;
            segmentRunning = true;
        }

        if (gamepad1.dpad_down && !manualRest && !segmentRunning) {
            sweeper.setPower(-1);
            sweep = -1;
        }if ((gamepad1.dpad_left || gamepad1.dpad_right) && !manualRest && !segmentRunning) {
            sweep = 0;
            sweepResetCheck = false;
        }
        if (gamepad1.dpad_up && !manualRest && !segmentRunning) {
            sweeper.setPower(1);
            sweep = 1;
        }

        if (sweep == 0 && !manualRest/* && !sweepResetCheck*/ && !segmentRunning) {
            int sr = sweeper.getCurrentPosition() % 720;
            if (sr < 0)
                sr = 720 - Math.abs(sr);
            if (sr <= 100 || sr >= 720) {
                sweeper.setPower(0);
                sweepResetCheck = true;
            } else if (sr <= 360) {
                sweeper.setPower(-.12);
            } else if (sr > 360) {
                sweeper.setPower(.12);
            }
        }
        else if (sweep == 0 && !manualRest && !segmentRunning)
            sweeper.setPower(0);

        if (segmentRunning) {
            telemetry.addData("Segment running for "+testTime+" ms", "Updating telemetry...");
            segmentRunning = !runSegment(testTime);
        } else {
            telemetry.addData("Encoder Value", sweeper.getCurrentPosition());
            telemetry.addData("Reset check", sweepResetCheck);
            telemetry.addData("Sweeper reset value", sweeper.getCurrentPosition() % 720);
            telemetry.addData("Sweeper reset range", "100 <= x <=719");
        }
    }

    public boolean runSegment(int t) {
        if (segmentTime + t > time) {
            sweeper.setPower(1);
            return false;
        }
        sweeper.setPower(0);
        return true;
    }
}
