package ftc.evlib.hardware.sensors;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.ValueHistory;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 3/16/16
 *
 * report the average of the last N values of a sensor as the value
 *
 * @see ftc.evlib.hardware.sensors.AnalogSensor
 */
public class AveragedSensor implements ftc.evlib.hardware.sensors.AnalogSensor {
    private final ftc.evlib.hardware.sensors.AnalogSensor sensor;
    private final ValueHistory valueHistory;
    private final Function scalingFunc;

    /**
     * @param sensor      the sensor to average
     * @param numReadings the number of readings to average
     */
    public AveragedSensor(ftc.evlib.hardware.sensors.AnalogSensor sensor, int numReadings) {
        this(sensor, numReadings, Functions.none());
    }

    public AveragedSensor(ftc.evlib.hardware.sensors.AnalogSensor sensor, int numReadings, Function scalingFunc) {
        this.sensor = sensor;
        valueHistory = new ValueHistory(numReadings);
        this.scalingFunc = scalingFunc;
    }

    /**
     * @return true once the first numReadings have been read
     */
    public boolean isReady() {
        return valueHistory.areAllActive();
    }

    @Override
    public Double getValue() {
        return valueHistory.getAverage();
    }

    public double getStandardDeviation() {
        return valueHistory.getStandardDeviation();
    }

    /**
     * read the sensor and update the average
     */
    public void act() {
        valueHistory.replaceOldestValue(scalingFunc.f(sensor.getValue()));
    }
}
