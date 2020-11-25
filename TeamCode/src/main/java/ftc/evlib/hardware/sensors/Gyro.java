package ftc.evlib.hardware.sensors;

/**
 * Created by ftc7393 on 12/9/2017.
 */

public interface Gyro {
    double getHeading();
//    void update();
    boolean isCalibrating();
    void stop();

    void setActive(boolean isActive);
}
