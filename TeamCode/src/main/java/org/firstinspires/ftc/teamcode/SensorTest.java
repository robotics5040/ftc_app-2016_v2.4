package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import java.util.Arrays;

/**
 * Created by bense on 2/13/2017.
 */
@Autonomous (name = "Sonar Test", group = "Testing")
@Disabled
public class SensorTest extends OpMode {
    I2cController c;
    I2cDevice sonar1;
    I2cDevice sonar2;
    I2cDeviceSynch sonar1Reader;
    I2cDeviceSynch sonar2Reader;
    byte[] sonar1Cache;
    byte[] sonar2Cache;

    public void init() {
        sonar1 = hardwareMap.i2cDevice.get("sonarRight");
        sonar1Reader = new I2cDeviceSynchImpl(sonar1, I2cAddr.create8bit(0x2c), false);
        sonar1Reader.engage();

        sonar2 = hardwareMap.i2cDevice.get("sonarLeft");
        sonar2Reader = new I2cDeviceSynchImpl(sonar2, I2cAddr.create8bit(0x2a), false);
        sonar2Reader.engage();
    }

    public void loop() {
        sonar1Cache = sonar1Reader.read(0x04, 1);
        sonar2Cache = sonar2Reader.read(0x04, 1);

        int us1 = sonar1Cache[0] & 0xff;
        int us2 = sonar2Cache[0] & 0xff;

        telemetry.addData("Sonar 1", us1);
        telemetry.addData("Sonar 2", us2);
    }
}
