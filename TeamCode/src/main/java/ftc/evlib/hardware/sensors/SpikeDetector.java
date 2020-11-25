package ftc.evlib.hardware.sensors;

import ftc.electronvolts.util.ValueHistory;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 1/18/17
 */

public class SpikeDetector implements ftc.evlib.hardware.sensors.DigitalSensor {

    private final ftc.evlib.hardware.sensors.AnalogSensor sensor;
    private final double spikeThreshold;

    private final ValueHistory shortHistory;
    private ValueHistory longHistory;
    private boolean isSpiking = false;

    /**
     * @param sensor           the AnalogSensor to detect spikes on
     * @param spikeThreshold   the percentage above the normal value that means there is a spike
     * @param numLongReadings  the number of readings to take for the long running average
     * @param numShortReadings the number of readings to take for the short running average
     */
    public SpikeDetector(ftc.evlib.hardware.sensors.AnalogSensor sensor, double spikeThreshold, int numLongReadings, int numShortReadings) {
        if (numShortReadings <= 0) {
            throw new IllegalArgumentException("numShortReadings (" + numShortReadings + ") cannot be less than or equal to 0");
        }
        if (numLongReadings <= 0) {
            throw new IllegalArgumentException("numLongReadings (" + numLongReadings + ") cannot be less than or equal to 0");
        }
        if (numShortReadings > numLongReadings) {
            throw new IllegalArgumentException("numShortReadings (" + numShortReadings + ") cannot be greater than numLongReadings (" + numLongReadings + ")");
        }

        this.sensor = sensor;
        this.spikeThreshold = spikeThreshold;
        longHistory = new ValueHistory(numLongReadings);
        shortHistory = new ValueHistory(numShortReadings);
    }

    public AnalogSensor getRawSensor() {
        return sensor;
    }

    /**
     * Update the averages and determine if a spike is happening
     */
    public void act() {
        //super awesome line that puts the new value in the short list, then takes the value from
        //the short list that was displaced by the new value and puts that into the long list
        longHistory.replaceOldestValue(shortHistory.replaceOldestValue(sensor.getValue()));

        //determine if a spike is happening
        if (longHistory.areAllActive()) {
            isSpiking = shortHistory.getAverage() >= (longHistory.getAverage() * spikeThreshold);
        }
    }
    public void resetSensor(){

        longHistory = new ValueHistory(longHistory.getLength());

    }


    public boolean isSpiking() {
        return isSpiking;
    }

    @Override
    public Boolean getValue() {
        return isSpiking;
    }

    public boolean isReady() {
        return longHistory.areAllActive();
    }

    public double getLongAverage() {
        return longHistory.getAverage();
    }

    public double getShortAverage() {
        return shortHistory.getAverage();
    }
}
