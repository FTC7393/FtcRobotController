package ftc.evlib.hardware.sensors;

import com.qualcomm.robotcore.hardware.GyroSensor;

import ftc.electronvolts.util.StateTimer;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 12/17/16
 */

public class GyroInitFixer implements GyroSensor {
    private final GyroSensor gyroSensor;
    private final long initTime;
    private final StateTimer stateTimer = new StateTimer();

    public GyroInitFixer(GyroSensor gyroSensor, long initTime) {
        this.gyroSensor = gyroSensor;
        this.initTime = initTime;
    }

    @Override
    public void calibrate() {
        stateTimer.init(initTime);
        gyroSensor.calibrate();
    }

    @Override
    public boolean isCalibrating() {
        return gyroSensor.isCalibrating() || !stateTimer.isDone();
    }

    //TODO reset the heading when the opmode starts to account for heading drift
    @Override
    public int getHeading() {
        return gyroSensor.getHeading();
    }

    @Override
    public double getRotationFraction() {
        return gyroSensor.getRotationFraction();
    }

    @Override
    public int rawX() {
        return gyroSensor.rawX();
    }

    @Override
    public int rawY() {
        return gyroSensor.rawY();
    }

    @Override
    public int rawZ() {
        return gyroSensor.rawZ();
    }

    @Override
    public void resetZAxisIntegrator() {
        gyroSensor.resetZAxisIntegrator();
    }

    @Override
    public String status() {
        return gyroSensor.status();
    }

    @Override
    public Manufacturer getManufacturer() {
        return gyroSensor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return gyroSensor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return gyroSensor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return gyroSensor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        gyroSensor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        gyroSensor.close();
    }
}
