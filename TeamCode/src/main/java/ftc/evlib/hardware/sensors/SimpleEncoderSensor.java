package ftc.evlib.hardware.sensors;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class SimpleEncoderSensor implements AnalogSensor {

    private final DcMotorEx motorWithEncoder;
    private double ticksAtLastReset;
    private double ticks;

    public SimpleEncoderSensor(DcMotorEx motorWithEncoder) {
        this.motorWithEncoder = motorWithEncoder;
    }

    /**
     * This method updates the encoder value, and stores it in a field for other methods to use.
     * IMPORTANT!!!!!!!!!
     * Only call this method ONE time per loop
     * We suggest that this is called in the pre_act() method of the robot config so that the sensor values are available for
     * any logic in the act() method
     */
    public void pre_act() {
        ticks = motorWithEncoder.getCurrentPosition() + 0.0;
    }

    @Override
    public Double getValue() {
        return ticks;
    }

    public void reset() {
        ticksAtLastReset = ticks;
    }

    public double getTicksSinceLastReset() {
        return  ticks - ticksAtLastReset;
    }
}
