package ftc.evlib.hardware.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 2/4/17
 *
 * @see I2cDevice
 */

public class RangeSensor {
    private static final I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    private final I2cDevice i2cDevice;
    private final I2cDeviceReader i2cDeviceReader;

    private int ultrasonicValue;
    private int opticalDistValue;

    private final AnalogSensor ultrasonicSensor = new AnalogSensor() {
        @Override
        public Double getValue() {
            return (double) ultrasonicValue;
        }
    };

    private final AnalogSensor opticalDistanceSensor = new AnalogSensor() {
        @Override
        public Double getValue() {
            return (double) opticalDistValue;
        }
    };

    /**
     * @param i2cDevice the i2c device from the hardwareMap
     */
    public RangeSensor(I2cDevice i2cDevice) {
        this.i2cDevice = i2cDevice;

        //create an i2c reader to read the register we want
        i2cDeviceReader = new I2cDeviceReader(i2cDevice, RANGE1ADDRESS, RANGE1_REG_START, RANGE1_READ_LENGTH);
    }

    public RangeSensor(HardwareMap hardwareMap, String hardwareName) {
        this(hardwareMap.i2cDevice.get(hardwareName));
    }

    public boolean act() {
        //do nothing if the i2c port is busy
        if (!i2cDevice.isI2cPortReady()) return false;

        //get the sensor reading
        byte[] buffer = i2cDeviceReader.getReadBuffer();

        if (buffer.length < 2) return false;

        ultrasonicValue = buffer[0] & 0xFF;
        opticalDistValue = buffer[1] & 0xFF;
        return true;
    }

    public int getUltrasonicValue() {
        return ultrasonicValue;
    }

    public int getOpticalDistValue() {
        return opticalDistValue;
    }

    public AnalogSensor getUltrasonicSensor() {
        return ultrasonicSensor;
    }

    public AnalogSensor getOpticalDistanceSensor() {
        return opticalDistanceSensor;
    }
}
