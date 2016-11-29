package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by bense on 11/7/2016.
 */
@Autonomous(name = "Color Test", group = "Testing")
@Disabled
public class ColorTest extends OpMode {
    ColorSensor color, line;
    DeviceInterfaceModule dim;
    long startTime, currentTime;
    boolean ledOn = false;

    public void init()
    {
        color = hardwareMap.colorSensor.get("color");
        I2cAddr newAddress = new I2cAddr(0x1d);
        color.setI2cAddress(newAddress);
        line = hardwareMap.colorSensor.get("line");
        dim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 2");
        startTime = System.currentTimeMillis();
        currentTime = System.currentTimeMillis();
        color.enableLed(false);
        line.enableLed(true);
        //dim.enableI2cReadMode(4, newAddress, 0x04, 2);
    }

    public void loop()
    {
        telemetry.addData("color", color.blue());
        telemetry.addData("line", line.blue());
        telemetry.addData("color address 7-bit", color.getI2cAddress().get7Bit());
        telemetry.addData("line address 7-bit", line.getI2cAddress().get7Bit());
        telemetry.addData("color address 8-bit", color.getI2cAddress().get8Bit());
        telemetry.addData("line address 8-bit", line.getI2cAddress().get8Bit());

        if (currentTime + 1000 < System.currentTimeMillis())
        {
            if (ledOn) {
                ledOn = false;
                color.enableLed(false);
                line.enableLed(true);
                dim.setLED(1, true);
            } else {
                ledOn = true;
                color.enableLed(true);
                line.enableLed(false);
                dim.setLED(1, false);
            }
            currentTime = System.currentTimeMillis();
        }
    }
}
