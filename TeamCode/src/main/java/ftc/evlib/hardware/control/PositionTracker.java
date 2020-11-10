package ftc.evlib.hardware.control;

import com.qualcomm.robotcore.hardware.GyroSensor;

import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 1/3/17
 */

public class PositionTracker {
    private final ftc.evlib.hardware.control.MecanumControl mecanumControl;
    private final GyroSensor gyro;

    private long lastTime = -1;
    private Distance x = Distance.zero();
    private Distance y = Distance.zero();

    public PositionTracker(ftc.evlib.hardware.control.MecanumControl mecanumControl, GyroSensor gyro) {
        this.mecanumControl = mecanumControl;
        this.gyro = gyro;
    }

    public void act() {
        long now = System.currentTimeMillis();
        if (lastTime < 0) lastTime = now;

        double scaleFactor = mecanumControl.getMecanumMotors().getScaleFactor();
        Vector2D movement = new Vector2D(
                mecanumControl.getVelocityX(),
                mecanumControl.getVelocityY()
        );
        Velocity maxRobotSpeed = mecanumControl.getMaxRobotSpeed(Angle.subtract(movement.getDirection(), Angle.fromDegrees(gyro.getHeading())));
        Time deltaTime = Time.fromMilliseconds(now - lastTime);
        x = Distance.add(x, Distance.fromMeters(movement.getX() * scaleFactor * maxRobotSpeed.getDistance(deltaTime).meters()));
        y = Distance.add(y, Distance.fromMeters(movement.getY() * scaleFactor * maxRobotSpeed.getDistance(deltaTime).meters()));

        lastTime = now;
    }

    public Distance getX() {
        return x;
    }

    public Distance getY() {
        return y;
    }
}
