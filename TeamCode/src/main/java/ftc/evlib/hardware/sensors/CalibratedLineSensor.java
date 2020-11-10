package ftc.evlib.hardware.sensors;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 3/8/16
 *
 * manages threshold calculation for the line sensor
 *
 * Note: this code is old and should be updated to use ValueHistory from the state-machine-framework
 *
 * @see ftc.evlib.hardware.sensors.AnalogSensor
 */
public class CalibratedLineSensor implements ftc.evlib.hardware.sensors.DigitalSensor {
    private boolean ready = true;
    private final ftc.evlib.hardware.sensors.AnalogSensor lineSensor;
    private static final int NUM_CALIBRATIONS = 100;
    private int numReadings;
    private double average, deviation, threshold = 80;
    private final double readings[] = new double[NUM_CALIBRATIONS];
    private boolean seeingLine = false;
    private double value;

    /**
     * @param lineSensor the raw line sensor
     */
    public CalibratedLineSensor(ftc.evlib.hardware.sensors.AnalogSensor lineSensor) {
        this.lineSensor = lineSensor;
    }

    /**
     * start calibrating the sensor
     */
    public void calibrate() {
        ready = false;
        average = 0;
        numReadings = 0;
        deviation = 0;
    }

    /**
     * @return true when done calibrating
     */
    public boolean isReady() {
        return ready;
    }

    public double getThreshold() {
        return threshold;
    }

    /**
     * @return true if the sensor is over the line
     */
    @Override
    public Boolean getValue() {
        return seeingLine;
    }

    /**
     * read the sensor value and use it to calibrate/update the average
     */
    public void act() {
        value = lineSensor.getValue();
        if (!ready) {
            if (numReadings < NUM_CALIBRATIONS) {
                //add a value
                readings[numReadings] = value;
                numReadings++;

                //update the average
                average = (average * (numReadings - 1) + value) / (numReadings);

            } else {
                //find deviation
                for (double reading : readings) {
                    deviation += (reading - average) * (reading - average);
                }
                deviation /= Math.sqrt(numReadings);

                threshold = (average - deviation) * (average - deviation) / average * 3.5;
                ready = true;
            }
        }
        seeingLine = value >= threshold;
    }

    /**
     * @return raw line sensor
     */
    public AnalogSensor getRawLineSensor() {
        return lineSensor;
    }
}
