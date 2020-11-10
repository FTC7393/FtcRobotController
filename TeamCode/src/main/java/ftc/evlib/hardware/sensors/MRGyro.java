package ftc.evlib.hardware.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.ResultReceiver;

/**
 * Created by ftc7393 on 12/9/2017.
 */

public class MRGyro implements Gyro {
    // The IMU sensor object
    final ModernRoboticsI2cGyro gyro;

    // State used for updating telemetry
//    Orientation angles=new Orientation();
//    Acceleration gravity;

    private final ResultReceiver<Double> angleReceiver;
    private final ResultReceiver<Boolean> stopReceiver;
    private final ResultReceiver<Boolean> isCalibrated = new BasicResultReceiver<>();

    private boolean isActive = false;

    public MRGyro(final ModernRoboticsI2cGyro gyro) {
        this.gyro = gyro;



        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "gyro".

        angleReceiver = new BasicResultReceiver<>();
        stopReceiver = new BasicResultReceiver<>();

        angleReceiver.setValue(0.0);
        Thread gyroReadThread = new Thread(new Runnable() {
            @Override
            public void run() {
                gyro.calibrate();
                isCalibrated.setValue(true);
                while (!stopReceiver.isReady()) {
                    if (isActive) {
                        double heading = gyro.getHeading();
                        angleReceiver.setValue(heading);
                        try{
                            Thread.sleep(10L);
                        } catch (InterruptedException e) {
                        }
                    } else {
                        try {
                            Thread.sleep(20L, 0);
                        } catch (InterruptedException e) {
                            // do nothing
                        }
                    }
                }

            }
        });
        gyroReadThread.start();


    }

//    protected void notSupported(){
//        throw new UnsupportedOperationException("Method not supported for "+getDeviceName());
//
//    }

    @Override
    public void setActive(boolean active) {
        this.isActive = active;
    }

    @Override
    public double getHeading() {
        return angleReceiver.getValue();
    }

//    @Override
//    public void update() {
//        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////        gravity  = imu.getGravity();
//    }

    @Override
    public boolean isCalibrating() {
        return !isCalibrated.isReady();

    }

    @Override
    public void stop() {
        stopReceiver.setValue(true);
    }
}
