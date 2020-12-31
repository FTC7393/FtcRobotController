package ftc.evlib.hardware.control;

import ftc.evlib.hardware.sensors.Gyro;
import ftc.electronvolts.util.ControlLoop;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.ProportionalController;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.units.Angle;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/20/16
 *
 * Factory class for RotationControl
 *
 * @see ftc.evlib.hardware.control.RotationControl
 */

public class RotationControls {

//    private static final double DEFAULT_GYRO_GAIN = 0.6; <- one possible good gyro gain, was used for a while

    /**
     * No movement
     */
    public static final ftc.evlib.hardware.control.RotationControl ZERO = constant(0);

    /**
     * rotate at a constant velocity
     *
     * @param velocityR the velocity to rotate at
     * @return the created RotationControl
     */
    public static ftc.evlib.hardware.control.RotationControl constant(final double velocityR) {
        return new ftc.evlib.hardware.control.RotationControl() {
            @Override
            public boolean act() {
                return true;
            }

            @Override
            public double getVelocityR() {
                return velocityR;
            }

            @Override
            public Angle getPolarDirectionCorrection() {
                return Angle.fromRadians(0);
            }
        };
    }

    /**
     * Rotate according to an InputExtractor's value (such as a driver joystick)
     *
     * @param rotation the InputExtractor
     * @return the created RotationControl
     */
    public static ftc.evlib.hardware.control.RotationControl inputExtractor(final InputExtractor<Double> rotation) {
        return new ftc.evlib.hardware.control.RotationControl() {
            @Override
            public boolean act() {
                return true;
            }

            @Override
            public double getVelocityR() {
                return rotation.getValue();
            }

            @Override
            public Angle getPolarDirectionCorrection() {
                return Angle.fromRadians(0);
            }
        };
    }

        HeadingSource gyroHeadingSrc = new HeadingSource(){
            @Override
            public double getHeading(){
                return gyro.getHeading();
            }
        }


        /**
         * Controls the rotation of a mecanum robot with a gyro sensor
         *
         * @param gyro            the gyro sensor
         * @param targetHeading   the direction to rotate to
         * @param tolerance       the deadzone
         * @param maxAngularSpeed the max speed to rotate at
         * @return the created RotationControl
         */
    public static ftc.evlib.hardware.control.RotationControl headingSource(final HeadingSource headingSrc, double gain, final Angle targetHeading, final Angle tolerance, final double maxAngularSpeed, final double minAngularSpeed) {
//        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile("AutoOptions.txt"));
//        double gain = optionsFile.get("gyro_gain", Double.class);
//        double max = optionsFile.get("gyro_max", Double.class);
//        final ControlLoop gyroControl = new ProportionalController(gain, 0.01, 0.05, max);

        //                                                         gain,      innerDeadzone,                outerDeadzone,             minOutput,       maxOutput
        final ControlLoop gyroControl = new ProportionalController(gain, tolerance.abs().radians(), minAngularSpeed, maxAngularSpeed);

//        final Vector2D targetHeadingVector = new Vector2D(1, Angle.multiply(targetHeading,-1));
        final Vector2D targetHeadingVector = new Vector2D(1, targetHeading);

        return new ftc.evlib.hardware.control.RotationControl() {
            private double gyroHeading, rotationCorrection;

            @Override
            public boolean act() {

                //get the gyro heading and convert it to a vector
                gyroHeading = gyro.getHeading();
                Vector2D gyroVector = new Vector2D(1, Angle.fromDegrees(gyroHeading));

                //find the "signed angular separation", the magnitude and direction of the error
                Angle signedAngularSeparation = Vector2D.signedAngularSeparation(targetHeadingVector, gyroVector);
//                telemetry.addData("signed angular separation", signedAngularSeparation.degrees());

//              This graph shows angle error vs. rotation correction
//              ____________________________
//              | correction.       ____   |
//              |           .      /       |
//              |           .   __/        |
//              | ........__.__|.......... |
//              |      __|  .     error    |
//              |     /     .              |
//              | ___/      .              |
//              |__________________________|
//
//              The code inside ProportionalController creates this graph

                //scale the signedAngularSeparation by a constant
                rotationCorrection = gyroControl.computeCorrection(0, signedAngularSeparation.radians());
//                telemetry.addData("rotationCorrection", rotationCorrection);

                return true;
            }

            @Override
            public double getVelocityR() {
                return rotationCorrection;
            }

            @Override
            public Angle getPolarDirectionCorrection() {
                return Angle.fromDegrees(-gyroHeading);
            }
        };
    }



    private enum TeleOpGyroMode {
        INIT,
        DRIVER,
        WAIT,
        GYRO
    }

    /**
     * Use driver input and do gyro stabilization when the input is zero
     *
     * @param driver          the driver input
     * @param gyro            the gyro to use for stabilization
     * @param maxAngularSpeed the maximum speed to rotate at when doing gyro stabilization
     * @return the created RotationControl
     */
    public static ftc.evlib.hardware.control.RotationControl teleOpGyro(final InputExtractor<Double> driver, final Gyro gyro,
                                                                        final Angle tolerance, final double maxAngularSpeed, final double gyroGain) {
        final long DELAY_BEFORE_GYRO_CONTROL = 500;
        final long INIT_TIME = 4000;

        final long startTime = System.currentTimeMillis();

       // gyro.calibrate();

        return new ftc.evlib.hardware.control.RotationControl() {

            //keeps track of who is in control: the driver or the gyro
            private TeleOpGyroMode mode = TeleOpGyroMode.INIT;

            //the output of this controller
            private double velocityR;

            //the RotationControl that uses the gyro
            private ftc.evlib.hardware.control.RotationControl gyroControl;

            private long driverEndTime;

            @Override
            public boolean act() {
//                Log.v("TeleOpGyro", "driver: " + driver.getValue() + "  mode: " + mode + "  gyroHeading: " + gyro.getHeading());

                velocityR = driver.getValue(); //get the value from the driver

                if (mode == TeleOpGyroMode.INIT) {
                    if (System.currentTimeMillis() - startTime < INIT_TIME) {
                        return true;
                    } else {
                        mode = TeleOpGyroMode.DRIVER;
                    }
                }
                //if the driver's input is 0, use the gyro control
                if (velocityR == 0) {
                    double gyroHeading = gyro.getHeading(); //get the gyro heading

                    //if the driver input just dropped to 0
                    if (mode == TeleOpGyroMode.DRIVER) {
                        mode = TeleOpGyroMode.WAIT;
                        driverEndTime = System.currentTimeMillis();
                    }
                    if (mode == TeleOpGyroMode.WAIT) {
                        if (System.currentTimeMillis() - driverEndTime >= DELAY_BEFORE_GYRO_CONTROL) {
                            mode = TeleOpGyroMode.GYRO;
                            //initialize the gyroControl
                            gyroControl = gyro(gyro, gyroGain, Angle.fromDegrees(gyroHeading), tolerance, maxAngularSpeed);
                        } else {
                            return true;
                        }
                    }
                    //update the gyro control
                    if (!gyroControl.act()) return false;

                    //use the gyro control's output
                    velocityR = -gyroControl.getVelocityR();
                } else {
                    mode = TeleOpGyroMode.DRIVER;
                }
                return true;
            }

            @Override
            public double getVelocityR() {
                return velocityR;
            }

            @Override
            public Angle getPolarDirectionCorrection() {
                //if the gyro is controlling
                if (mode == TeleOpGyroMode.GYRO) {
                    //use the gyro heading to correct for the angle
                    return gyroControl.getPolarDirectionCorrection();
                } else {
                    //otherwise, don't apply a correction to the translation angle
                    return Angle.zero();
                }
            }
        };
    }

    private static class HeadingSource {
    }
}
