package ftc.evlib.hardware.control;

import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.units.Angle;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 12/14/16
 *
 * Factory class for the XYR abstract class
 *
 * @see ftc.evlib.hardware.control.XYRControl
 */

public class XYRControls {
    /**
     * No movement
     */
    public static final ftc.evlib.hardware.control.XYRControl ZERO = constant(0, Angle.zero(), 0);

//    public static XYRControl combined(final XYRControl xyrControl1, final XYRControl xyrControl2) {
//        return new XYRControl() {
//            double velocityR = 0;
//            Vector2D translation = new Vector2D(0, 0);
//
//            @Override
//            public boolean act() {
//                boolean worked = xyrControl1.act() && xyrControl2.act();
//                velocityR = xyrControl1.getVelocityR() + xyrControl2.getVelocityR();
//                Vector2D translation1 = xyrControl1.getTranslation();
//                Vector2D translation2 = xyrControl2.getTranslation();
//                translation = new Vector2D(
//                        translation1.getX() + translation2.getX(),
//                        translation1.getY() + translation2.getY()
//                );
//                return worked;
//            }
//
//            @Override
//            public double getVelocityR() {
//                return velocityR;
//            }
//
//            @Override
//            public Vector2D getTranslation() {
//                return translation;
//            }
//        };
//    }

    /**
     * Constant velocity, direction, and rotation
     *
     * @param velocity  how fast to move from -1 to 1
     * @param direction what direction to move
     * @param rotation  how fast to rotate from -1 to 1
     * @return the created XYRControl
     */
    public static ftc.evlib.hardware.control.XYRControl constant(double velocity, Angle direction, final double rotation) {
        final Vector2D vector2D = new Vector2D(velocity, direction);

        return new ftc.evlib.hardware.control.XYRControl() {
            @Override
            public boolean act() {
                return true;
            }

            @Override
            public double getVelocityR() {
                return rotation;
            }

            @Override
            public Vector2D getTranslation() {
                return vector2D;
            }
        };
    }

    /**
     * Use InputExtractor objects to determine the X, Y, and R velocities
     *
     * @param velocity         the InputExtractor that gives the magnitude of the velocity
     * @param directionDegrees the InputExtractor that gives the direction in degrees
     * @param rotation         the InputExtractor that gives the rotation speed from -1 to 1
     * @return the created XYRControl
     */
    public static ftc.evlib.hardware.control.XYRControl inputExtractorPolarDegreesR(final InputExtractor<Double> velocity, final InputExtractor<Double> directionDegrees, final InputExtractor<Double> rotation) {
        return new ftc.evlib.hardware.control.XYRControl() {
            @Override
            public boolean act() {
                return false;
            }

            @Override
            public double getVelocityR() {
                return rotation.getValue();
            }

            @Override
            public Vector2D getTranslation() {
                return new Vector2D(velocity.getValue(), Angle.fromDegrees(directionDegrees.getValue()));
            }
        };
    }


    /**
     * Use InputExtractor objects to determine the X, Y, and R velocities
     *
     * @param velocity  the InputExtractor that gives the magnitude of the velocity
     * @param direction the InputExtractor that gives the direction
     * @param rotation  the InputExtractor that gives the rotation speed from -1 to 1
     * @return the created XYRControl
     */
    public static ftc.evlib.hardware.control.XYRControl inputExtractorPolarR(final InputExtractor<Double> velocity, final InputExtractor<Angle> direction, final InputExtractor<Double> rotation) {
        return new ftc.evlib.hardware.control.XYRControl() {
            @Override
            public boolean act() {
                return false;
            }

            @Override
            public double getVelocityR() {
                return rotation.getValue();
            }

            @Override
            public Vector2D getTranslation() {
                return new Vector2D(velocity.getValue(), direction.getValue());
            }
        };
    }

    /**
     * Use InputExtractor objects to determine the X, Y, and R velocities
     *
     * @param velocityX the InputExtractor that gives the magnitude of the velocity in the X direction
     * @param velocityY the InputExtractor that gives the magnitude of the velocity in the Y direction
     * @param velocityR the InputExtractor that gives the magnitude of the rotational velocity
     * @return the created XYRControl
     */
    public static ftc.evlib.hardware.control.XYRControl inputExtractorXYR(final InputExtractor<Double> velocityX, final InputExtractor<Double> velocityY, final InputExtractor<Double> velocityR) {
        return new ftc.evlib.hardware.control.XYRControl() {
            @Override
            public boolean act() {
                return false;
            }

            @Override
            public double getVelocityR() {
                return velocityR.getValue();
            }

            @Override
            public Vector2D getTranslation() {
                return new Vector2D(velocityX.getValue(), velocityY.getValue());
            }
        };
    }


    /**
     * Use InputExtractor objects to determine the X, Y, and R velocities
     *
     * @param velocityXY the InputExtractor that gives the magnitude of the velocity in the X and Y directions
     * @param velocityR  the InputExtractor that gives the magnitude of the rotational velocity
     * @return the created XYRControl
     */
    public static ftc.evlib.hardware.control.XYRControl inputExtractorXYR(final InputExtractor<Vector2D> velocityXY, final InputExtractor<Double> velocityR) {
        return new ftc.evlib.hardware.control.XYRControl() {
            @Override
            public boolean act() {
                return false;
            }

            @Override
            public double getVelocityR() {
                return velocityR.getValue();
            }

            @Override
            public Vector2D getTranslation() {
                return velocityXY.getValue();
            }
        };
    }
}
