package ftc.evlib.hardware.sensors;

import ftc.evlib.util.ImmutableList;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 10/31/16
 *
 * Implementation of LineSensorArray that combines two SingleLineSensorArray objects
 *
 * @see SingleLineSensorArray
 * @see ftc.evlib.hardware.sensors.LineSensorArray
 */
public class DoubleLineSensorArray extends ftc.evlib.hardware.sensors.NLineSensorArray {
    public DoubleLineSensorArray(ftc.evlib.hardware.sensors.LineSensorArray leftLineSensorArray, LineSensorArray rightLineSensorArray) {
        super(ImmutableList.of(leftLineSensorArray, rightLineSensorArray));
    }
}
