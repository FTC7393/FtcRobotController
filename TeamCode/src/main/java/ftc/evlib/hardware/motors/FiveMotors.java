package ftc.evlib.hardware.motors;

import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.util.ImmutableList;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/12/16
 *
 * A subclass of NMotors that provides convenience methods for passing in 5 motor powers.
 *
 * @see ftc.evlib.hardware.motors.NMotors
 */
public class FiveMotors extends ftc.evlib.hardware.motors.NMotors {
    public FiveMotors(ftc.evlib.hardware.motors.Motor motor1, ftc.evlib.hardware.motors.Motor motor2, ftc.evlib.hardware.motors.Motor motor3, ftc.evlib.hardware.motors.Motor motor4, Motor motor5, boolean useSpeedMode, Velocity maxRobotSpeed) {
        super(ImmutableList.of(motor1, motor2, motor3, motor4, motor5), useSpeedMode, maxRobotSpeed);
    }

    public void runMotorsNormalized(double value1, double value2, double value3, double value4, double value5) {
        runNormalized(ImmutableList.of(value1, value2, value3, value4, value5));
    }

    public void runMotors(double value1, double value2, double value3, double value4, double value5) {
        run(ImmutableList.of(value1, value2, value3, value4, value5));
    }
}
