package ftc.evlib.hardware.motors;

import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.util.ImmutableList;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 12/27/15
 *
 * A subclass of NMotors that provides convenience methods for passing in 6 motor powers.
 *
 * @see ftc.evlib.hardware.motors.NMotors
 */
public class SixMotors extends ftc.evlib.hardware.motors.NMotors {
    public SixMotors(ftc.evlib.hardware.motors.Motor frontLeftMotor, ftc.evlib.hardware.motors.Motor frontRightMotor, ftc.evlib.hardware.motors.Motor middleLeftMotor, ftc.evlib.hardware.motors.Motor middleRightMotor, ftc.evlib.hardware.motors.Motor backLeftMotor, Motor backRightMotor, boolean useSpeedMode, Velocity maxRobotSpeed) {
        super(ImmutableList.of(frontLeftMotor, frontRightMotor, middleLeftMotor, middleRightMotor, backLeftMotor, backRightMotor), useSpeedMode, maxRobotSpeed);
    }

    public void runMotorsNormalized(double flValue, double frValue, double mlValue, double mrValue, double blValue, double brValue) {
        runNormalized(ImmutableList.of(flValue, frValue, mlValue, mrValue, blValue, brValue));
    }

    public void runMotors(double flValue, double frValue, double mlValue, double mrValue, double blValue, double brValue) {
        run(ImmutableList.of(flValue, frValue, mlValue, mrValue, blValue, brValue));
    }
}
