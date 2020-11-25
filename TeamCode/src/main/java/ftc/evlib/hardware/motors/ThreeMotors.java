package ftc.evlib.hardware.motors;

import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.util.ImmutableList;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/12/16
 *
 * A subclass of NMotors that provides convenience methods for passing in 3 motor powers.
 *
 * @see ftc.evlib.hardware.motors.NMotors
 */
public class ThreeMotors extends ftc.evlib.hardware.motors.NMotors {
    public ThreeMotors(ftc.evlib.hardware.motors.Motor motor1, ftc.evlib.hardware.motors.Motor motor2, Motor motor3, boolean useSpeedMode, Velocity maxRobotSpeed) {
        super(ImmutableList.of(motor1, motor2, motor3), useSpeedMode, maxRobotSpeed);
    }

    public void runMotorsNormalized(double value1, double value2, double value3) {
        runNormalized(ImmutableList.of(value1, value2, value3));
    }

    public void runMotors(double value1, double value2, double value3) {
        run(ImmutableList.of(value1, value2, value3));
    }
}
