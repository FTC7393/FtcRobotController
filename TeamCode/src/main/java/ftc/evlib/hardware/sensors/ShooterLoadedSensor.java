package ftc.evlib.hardware.sensors;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import java.util.Objects;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.electronvolts.util.TeamColor;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 2/2/17
 */

public class ShooterLoadedSensor implements ftc.evlib.hardware.sensors.DigitalSensor {
    private static final long DEBOUNCE_TIME = 500;

    private final DeviceInterfaceModule deviceInterfaceModule;
    private final RangeSensor rangeSensor;
    private final int ultrasonicThreshold, opticalDistThreshold;

    private boolean hasParticle = false;
    private TeamColor particleColorRaw = TeamColor.UNKNOWN;
    private TeamColor particleColor = TeamColor.UNKNOWN;
    private long debounceTimer = 0;

    public ShooterLoadedSensor(DeviceInterfaceModule deviceInterfaceModule, RangeSensor rangeSensor, int ultrasonicThreshold, int opticalDistThreshold) {
        this.deviceInterfaceModule = deviceInterfaceModule;
        this.rangeSensor = rangeSensor;
        this.ultrasonicThreshold = ultrasonicThreshold;
        this.opticalDistThreshold = opticalDistThreshold;
    }

    public void act() {
        rangeSensor.act();
        int ultrasonicValue = rangeSensor.getUltrasonicValue();
        int opticalDistValue = rangeSensor.getOpticalDistValue();

        TeamColor lastParticleColorRaw = particleColorRaw;

        hasParticle = ultrasonicValue <= ultrasonicThreshold;

        if (hasParticle) {
            if (opticalDistValue > opticalDistThreshold) {
                particleColorRaw = TeamColor.RED;
            } else {
                particleColorRaw = TeamColor.BLUE;
            }
        } else {
            particleColorRaw = TeamColor.UNKNOWN;
            particleColor = TeamColor.UNKNOWN;
        }

        long now = System.currentTimeMillis();

        if (!Objects.equals(lastParticleColorRaw, particleColorRaw)) {
            debounceTimer = now;
        }

        if (now - debounceTimer >= DEBOUNCE_TIME) {
            particleColor = particleColorRaw;
        }

        deviceInterfaceModule.setLED(RobotCfg.DEVICE_MODULE_RED_LED, particleColor == TeamColor.RED);
        deviceInterfaceModule.setLED(RobotCfg.DEVICE_MODULE_BLUE_LED, particleColor == TeamColor.BLUE);
    }

    public boolean hasParticle() {
        return hasParticle;
    }

    @Override
    public Boolean getValue() {
        return hasParticle;
    }

    public TeamColor getParticleColorRaw() {
        return particleColorRaw;
    }

    public TeamColor getParticleColor() {
        return particleColor;
    }

    /**
     * Used for when the shooter shoots to reset the particle color
     *
     * @param particleColor which color the particle is from an external source
     */
    public void setParticleColor(TeamColor particleColor) {
        this.particleColor = particleColor;
    }

    public boolean isIncorrectColor(TeamColor teamColor) {
        return !Objects.equals(particleColor, TeamColor.UNKNOWN) && (teamColor == TeamColor.RED || teamColor == TeamColor.BLUE) && !Objects.equals(particleColor, teamColor);
    }

    public RangeSensor getRangeSensor() {
        return rangeSensor;
    }
}
