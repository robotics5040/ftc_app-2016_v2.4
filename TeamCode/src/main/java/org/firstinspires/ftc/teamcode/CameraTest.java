package org.firstinspires.ftc.teamcode;

import android.location.Location;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Trackable;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcontroller.internal.testcode.FTCVuforia;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.HashMap;

/**
 * Created by bense on 9/12/2016.
 */
@TeleOp(name = "CameraTest", group = "Tests")
public class CameraTest extends OpMode {
    FTCVuforia vuforia;
    public void init()
    {
        vuforia = FtcRobotControllerActivity.getVuforia();
        vuforia.addTrackables("tphone.xml");
        vuforia.initVuforia();
    }

    public void loop()
    {
        HashMap<String, double[]> data = vuforia.getVuforiaData();

        if (data.containsKey("tphone"))
        {
            telemetry.addData("Phone", data.get("tphone"));
        }
    }

    public void stop()
    {
        super.stop();
        try {
            vuforia.destroy();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
