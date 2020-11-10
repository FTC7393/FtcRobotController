package ftc.evlib.hardware.motors;

import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.util.ImmutableList;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/12/16
 *
 * A subclass of NMotors that provides convenience methods for passing in 2 motor powers.
 *
 * @see ftc.evlib.hardware.motors.NMotors
 */
public class TwoMotors extends ftc.evlib.hardware.motors.NMotors {
    public TwoMotors(ftc.evlib.hardware.motors.Motor leftMotor, Motor rightMotor, boolean useSpeedMode, Velocity maxRobotSpeed) {
        super(ImmutableList.of(leftMotor, rightMotor), useSpeedMode, maxRobotSpeed);
    }

    public void runMotorsNormalized(double leftValue, double rightValue) {
        runNormalized(ImmutableList.of(leftValue, rightValue));
    }

    public void runMotors(double leftValue, double rightValue) {
        run(ImmutableList.of(leftValue, rightValue));
    }
}
