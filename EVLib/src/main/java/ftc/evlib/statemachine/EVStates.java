package ftc.evlib.statemachine;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.List;
import java.util.Map;

import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.control.RotationControl;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControl;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.hardware.motors.MecanumMotors;
import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.motors.NMotors;
import ftc.evlib.hardware.motors.TwoMotors;
import ftc.evlib.hardware.sensors.DigitalSensor;
import ftc.evlib.hardware.sensors.DoubleLineSensor;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.Servos;
//import ftc.evlib.vision.framegrabber.VuforiaFrameFeeder;
//import ftc.evlib.vision.processors.BeaconColorResult;
//import ftc.evlib.vision.processors.BeaconName;
//import ftc.evlib.vision.processors.CloseUpColorProcessor;
//import ftc.evlib.vision.processors.ImageProcessor;
//import ftc.evlib.vision.processors.ImageProcessorResult;
//import ftc.evlib.vision.processors.Particle;
//import ftc.evlib.vision.processors.ParticleFinder;
//import ftc.evlib.vision.processors.Particles;
//import ftc.evlib.vision.processors.RGBBeaconProcessor;
//import ftc.evlib.vision.processors.VuforiaBeaconColorProcessor;
import ftc.electronvolts.statemachine.AbstractState;
import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.EndConditions;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMap;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.statemachine.States;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.Utility;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;

import static ftc.evlib.driverstation.Telem.telemetry;
//import static ftc.evlib.vision.framegrabber.GlobalFrameGrabber.frameGrabber;
//import static ftc.evlib.vision.framegrabber.VuforiaFrameFeeder.beacons;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 5/10/16
 * Change the mecanum drive and gyro turn to max angular speed
 * @see State
 * @see EVStateMachineBuilder
 */
public class EVStates extends States {
//    public static State findParticlesState(final StateName nextStateName, final StateName timeoutState, Time timeout, final Particles particles, final boolean saveImages, final ServoControl phoneServo, final Function degreesFromServoPos) {
//        final EndCondition timeoutEC = EVEndConditions.timed(timeout);
//
//        return new BasicAbstractState() {
//            private boolean timedOut = false;
//
//            @Override
//            public void init() {
//                frameGrabber.setImageProcessor(new ParticleFinder());
//                frameGrabber.setSaveImages(saveImages);
//                frameGrabber.grabSingleFrame();
//                timeoutEC.init();
//            }
//
//            @Override
//            public boolean isDone() {
//                if (frameGrabber.isResultReady()) {
//                    return true;
//                }
//
//                timedOut = timeoutEC.isDone();
//                return timedOut;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                if (timedOut) {
//                    return timeoutState;
//                } else {
//                    List<Particle> results = (List<Particle>) frameGrabber.getResult().getResult();
//                    Log.i("Particle", "servo: " + degreesFromServoPos.f(phoneServo.getCurrentPosition()) + " degrees");
//                    particles.add(results, Angle.fromDegrees(degreesFromServoPos.f(phoneServo.getCurrentPosition())));
//                    return nextStateName;
//                }
//            }
//        };
//    }
//    public static State findParticlesState(final StateName nextStateName, final StateName timeoutState, Time timeout, final Particles particles, final boolean saveImages) {
//        final EndCondition timeoutEC = EVEndConditions.timed(timeout);
//
//        return new BasicAbstractState() {
//            private boolean timedOut = false;
//
//            @Override
//            public void init() {
//                frameGrabber.setImageProcessor(new ParticleFinder());
//                frameGrabber.setSaveImages(saveImages);
//                frameGrabber.grabSingleFrame();
//                timeoutEC.init();
//            }
//
//            @Override
//            public boolean isDone() {
//                if (frameGrabber.isResultReady()) {
//                    return true;
//                }
//
//                timedOut = timeoutEC.isDone();
//                return timedOut;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                if (timedOut) {
//                    return timeoutState;
//                } else {
//                    List<Particle> results = (List<Particle>) frameGrabber.getResult().getResult();
//                    return nextStateName;
//                }
//            }
//        };
//    }
//
//    public static State turnToFaceParticle(final StateName successState, final StateName noParticlesFoundState, final TeamColor teamColor, final Particles particles, final MecanumControl mecanumControl, final Gyro gyro, final Angle tolerance, final double maxAngularSpeed) {
//        return new State() {
//            boolean first = true;
//            private EndCondition gyroEC;
//
//            @Override
//            public StateName act() {
//                if (first) {
//                    List<Vector2D> movements = particles.getMovementsToParticle(teamColor);
//                    if (movements.isEmpty()) {
//                        return noParticlesFoundState;
//                    }
//                    first = false;
//
//                    Angle direction = movements.get(0).getDirection();
//
//                    //add a certain amount to the direction based on the team color because they have different starting directions
//                    direction = Angle.add(direction, teamColor == TeamColor.RED ? Angle.fromDegrees(90) : Angle.fromDegrees(180));
//
//                    mecanumControl.setControl(
//                            TranslationControls.ZERO,
//                            RotationControls.gyro(gyro, direction, tolerance, maxAngularSpeed)
//                    );
//                    gyroEC = EVEndConditions.gyroCloseTo(gyro, direction, tolerance);
//                    gyroEC.init();
//                }
//                if (gyroEC.isDone()) {
//                    mecanumControl.stop();
//                    first = true;
//                    return successState;
//                }
//                return null;
//            }
//        };
//    }
//
//
//    public static State collectParticle(final StateName successState, final StateName noParticlesFoundState, final TeamColor teamColor, final Particles particles, final MecanumControl mecanumControl, final Motor collector, final Gyro gyro, final Angle tolerance, final boolean turnCollectorOff, final double maxAngularSpeed) {
//        final double velocity = 1;
//        final double collectorPower = 1;
//        final Angle largeTolerance = Angle.fromDegrees(15);
//
//        return new State() {
//            boolean newMovement = true;
//            boolean first = true;
//
//            List<Vector2D> movements;
//            int movementIndex = 0;
//            Angle direction;
//            Distance distance;
////            EndCondition timeEC;
//
//            Distance distanceTravelled = Distance.zero();
//            long lastTime = 0;
//
//            @Override
//            public StateName act() {
//                if (first) {
//                    movements = particles.getMovementsToParticle(teamColor);
//                    if (movements.size() == 0) {
//                        return noParticlesFoundState;
//                    }
//                    collector.setPower(-collectorPower);
//                    first = false;
//                    newMovement = true;
//                }
//                if (newMovement) {
//                    newMovement = false;
//                    direction = movements.get(movementIndex).getDirection();
//                    distance = Distance.fromInches(movements.get(movementIndex).getLength());
////                    double speedMetersPerMillisecond = mecanumControl.getMaxRobotSpeed(Angle.zero()).metersPerMillisecond() * velocity;
////                    double durationMillis = Math.abs(distance.meters() / speedMetersPerMillisecond);
////                    timeEC = EVEndConditions.timed((long) durationMillis);
//
//                    distanceTravelled = Distance.zero();
//                    lastTime = System.currentTimeMillis();
//
////                    timeEC.init();
//                    mecanumControl.setControl(
////                            TranslationControls.ZERO,
//                            TranslationControls.constant(velocity, direction),
//                            RotationControls.gyro(gyro, direction, tolerance, maxAngularSpeed)
//                    );
//                    if (movementIndex == movements.size() - 1) {
//                        collector.setPower(collectorPower);
//                    }
//                }
//
//                Angle signedAngularSeparation = Vector2D.signedAngularSeparation(
//                        new Vector2D(1, Angle.fromDegrees(gyro.getHeading())),
//                        new Vector2D(1, direction)
//                );
//                if (signedAngularSeparation.abs().radians() <= largeTolerance.radians()) {
//                    mecanumControl.setTranslationControl(TranslationControls.constant(velocity, direction));
//                }
//
//                long now = System.currentTimeMillis();
//                Time deltaTime = Time.fromMilliseconds(lastTime - now);
//                lastTime = now;
//                distanceTravelled = Distance.add(
//                        distanceTravelled,
//                        Distance.multiply(
//                                mecanumControl.getMaxRobotSpeed(Angle.subtract(direction, Angle.fromDegrees(gyro.getHeading()))).getDistance(deltaTime),
//                                velocity * mecanumControl.getMecanumMotors().getScaleFactor()
//                        ).abs()
//                );
//                if (distanceTravelled.meters() >= distance.meters()) {
////                if (timeEC.isDone()) {
//                    newMovement = true;
//                    movementIndex++;
//                    if (movementIndex >= movements.size()) {
//                        if (turnCollectorOff) {
//                            collector.setPower(0);
//                        }
//                        mecanumControl.stop();
//                        first = true;
//                        return successState;
//                    }
//                }
//                return null;
//            }
//        };
//    }
//
//    public static State displayParticles(final Particles particles) {
//        return new State() {
//            @Override
//            public StateName act() {
//                StringBuilder sb = new StringBuilder();
//                for (Particle particle : particles) {
//                    sb.append(particle).append(" ");
//                }
//                telemetry.addData("Particles", sb);
//                return null;
//            }
//        };
//    }
//
//    /**
//     * Displays the left and right color of a BeaconColorResult
//     *
//     * @param receiver the ResultReceiver to get the color from
//     * @return the created State
//     * @see BeaconColorResult
//     */
//    public static State displayBeaconColorResult(final ResultReceiver<BeaconColorResult> receiver) {
//        return new BasicAbstractState() {
//            @Override
//            public void init() {
//
//            }
//
//            @Override
//            public boolean isDone() {
////                Telem.displayBeaconColorResult(receiver);
//                return false;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                return null;
//            }
//        };
//    }
//
//    /**
//     * Displays the color of a BeaconColorResult.BeaconColor
//     *
//     * @param receiver the ResultReceiver to get the color from
//     * @return the created State
//     * @see BeaconColorResult
//     */
////    public static State displayBeaconColor(final ResultReceiver<BeaconColorResult.BeaconColor> receiver) {
////        return new BasicAbstractState() {
////            @Override
////            public void init() {
////
////            }
////
////            @Override
////            public boolean isDone() {
////                Telem.displayBeaconColor(receiver);
////                return false;
////            }
////
////            @Override
////            public StateName getNextStateName() {
////                return null;
////            }
////        };
////    }
//
//    /**
//     * Uses vuforia to find the beacon target image, then uses opencv to determine the beacon color
//     *
//     * @param successState      the state to go to if it succeeds
//     * @param failState         the state to go to if it fails
//     * @param timeoutState      the state to go to if it times out
//     * @param timeoutTime       the time before it will time out
//     * @param vuforiaReceiver   the ResultReceiver to get the VuforiaFramFeeder object from
//     * @param beaconColorResult the ResultReceiver to store the result in
//     * @param teamColor         your team's color to decide which beacons to look for
//     * @param numFrames         the number of frames to process
//     * @param saveImages        whether or not to save the frames for logging
//     * @return the created State
//     * @see VuforiaFrameFeeder
//     * @see VuforiaBeaconColorProcessor
//     */
//    //TODO assume that vuforia is initialized in findBeaconColorState
//    public static State findBeaconColorState(final StateName successState, final StateName failState, final StateName timeoutState, Time timeoutTime, final ResultReceiver<VuforiaFrameFeeder> vuforiaReceiver, final ResultReceiver<BeaconColorResult> beaconColorResult, TeamColor teamColor, final int numFrames, final boolean saveImages) {
//        final List<BeaconName> beaconNames = BeaconName.getNamesForTeamColor(teamColor);
//        final EndCondition timeout = EndConditions.timed(timeoutTime);
//
//        return new BasicAbstractState() {
//            private VuforiaFrameFeeder vuforia = null;
//            private VuforiaBeaconColorProcessor processor = null;
//            private BeaconName beaconName;
//            private int beaconIndex = 0; //index of the beaconNames list
//            private boolean timedOut = false;
//
//            @Override
//            public void init() {
//                timeout.init();
//                timedOut = false;
//
//                if (beaconIndex >= beaconNames.size()) {
//                    beaconIndex = 0;
//                    //we should never go here
//                }
//                beaconName = beaconNames.get(beaconIndex);
//                if (processor != null) {
//                    processor.setBeaconName(beaconName);
//                }
//            }
//
//            @Override
//            public boolean isDone() {
//                if (vuforia == null && vuforiaReceiver.isReady()) {
//                    vuforia = vuforiaReceiver.getValue();
////                    if (vuforia == null) {
////                        Log.e("EVStates", "vuforia is null!!!!!!!!!!!!!");
////                    }
//
//                    processor = new VuforiaBeaconColorProcessor(vuforia);
//                    processor.setBeaconName(beaconName);
//
//                    VuforiaTrackable beacon = beacons.get(beaconName);
//                    beacon.getTrackables().activate();
//
//                    frameGrabber.setImageProcessor(processor);
//                    frameGrabber.setSaveImages(saveImages);
//                    frameGrabber.grabContinuousFrames();
//                }
//                timedOut = timeout.isDone();
//                return timedOut || processor != null && processor.getResultsFound() >= numFrames;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                beaconIndex++;
//                frameGrabber.stopFrameGrabber();
//                VuforiaTrackable beacon = beacons.get(beaconName);
//                beacon.getTrackables().deactivate();
//
//                BeaconColorResult result = processor.getAverageResult();
//                processor.reset();
//                BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
//                BeaconColorResult.BeaconColor rightColor = result.getRightColor();
//                if ((leftColor == BeaconColorResult.BeaconColor.RED && rightColor == BeaconColorResult.BeaconColor.BLUE)
//                        || (leftColor == BeaconColorResult.BeaconColor.BLUE && rightColor == BeaconColorResult.BeaconColor.RED)) {
//                    beaconColorResult.setValue(result);
//                    return successState;
//                } else {
//                    beaconColorResult.setValue(new BeaconColorResult());
//                    if (timedOut) {
//                        return timeoutState;
//                    } else {
//                        return failState;
//                    }
//                }
//            }
//        };
//    }
//
//    public static State findColorState(final StateName redState, final StateName blueState, final StateName unknownState, final Time timeout, final ResultReceiver<BeaconColorResult.BeaconColor> colorResult, final boolean saveImages) {
//        final EndCondition timeoutEC = EVEndConditions.timed(timeout);
//        return new BasicAbstractState() {
//            private boolean timedOut = false;
//
//            @Override
//            public void init() {
//                frameGrabber.setImageProcessor(new CloseUpColorProcessor());
//                frameGrabber.setSaveImages(saveImages);
//                frameGrabber.grabSingleFrame();
//                timeoutEC.init();
//            }
//
//            @Override
//            public boolean isDone() {
//                if (frameGrabber.isResultReady()) {
//                    return true;
//                }
//
//                timedOut = timeoutEC.isDone();
//                return timedOut;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                if (timedOut) {
//                    return unknownState;
//                } else {
//                    BeaconColorResult.BeaconColor result = (BeaconColorResult.BeaconColor) frameGrabber.getResult().getResult();
//                    colorResult.setValue(result);
//                    switch (result) {
//                        case RED:
//                            return redState;
//                        case BLUE:
//                            return blueState;
//                        default:
//                            return unknownState;
//                    }
//                }
//            }
//        };
//    }
//
//    public static State beaconColorSwitch(final StateName redState, final StateName blueState, final StateName unknownState, ResultReceiver<BeaconColorResult.BeaconColor> colorResult) {
//        return new BasicAbstractState() {
//            @Override
//            public void init() {
//
//            }
//
//            @Override
//            public boolean isDone() {
//                return true;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                BeaconColorResult.BeaconColor result = (BeaconColorResult.BeaconColor) frameGrabber.getResult().getResult();
//                switch (result) {
//                    case RED:
//                        return redState;
//                    case BLUE:
//                        return blueState;
//                    default:
//                        return unknownState;
//                }
//            }
//        };
//    }
//
//    /**
//     * @param nextStateName  the name of the next state
//     * @param imageProcessor the object that processes the image
//     * @param resultReceiver the object that stores the image
//     * @return the created State
//     */
//    public static State processFrame(final StateName nextStateName, final ImageProcessor imageProcessor, final ResultReceiver<ImageProcessorResult> resultReceiver) {
//        return new BasicAbstractState() {
//            @Override
//            public void init() {
//                frameGrabber.setImageProcessor(imageProcessor);
//                frameGrabber.grabSingleFrame();
//            }
//
//            @Override
//            public boolean isDone() {
//                return frameGrabber.isResultReady();
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                resultReceiver.setValue(frameGrabber.getResult());
//                return nextStateName;
//            }
//        };
//    }
//
//    /**
//     * @param unknownUnknownState if both sides are unknown
//     * @param unknownRedState     if the left is unknown and the right is red
//     * @param unknownBlueState    if the left is unknown and the right is blue
//     * @param redUnknownState     if the left is red and the right is unknown
//     * @param redRedState         if the left is red and the right is red
//     * @param redBlueState        if the left is red and the right is blue
//     * @param blueUnknownState    if the left is blue and the right is unknown
//     * @param blueRedState        if the left is blue and the right is red
//     * @param blueBlueState       if the left is blue and the right is blue
//     * @return the created State
//     */
//    public static State processBeaconPicture(
//            final StateName unknownUnknownState, final StateName unknownRedState, final StateName unknownBlueState,
//            final StateName redUnknownState, final StateName redRedState, final StateName redBlueState,
//            final StateName blueUnknownState, final StateName blueRedState, final StateName blueBlueState
//    ) {
//
//        return new BasicAbstractState() {
//            @Override
//            public void init() {
//                frameGrabber.setImageProcessor(new RGBBeaconProcessor());
//                frameGrabber.grabSingleFrame();
//            }
//
//            @Override
//            public boolean isDone() {
//                return frameGrabber.isResultReady();
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                BeaconColorResult beaconColorResult = (BeaconColorResult) frameGrabber.getResult().getResult();
//                BeaconColorResult.BeaconColor leftColor = beaconColorResult.getLeftColor();
//                BeaconColorResult.BeaconColor rightColor = beaconColorResult.getRightColor();
//
//                if (leftColor == BeaconColorResult.BeaconColor.RED) {
//                    if (rightColor == BeaconColorResult.BeaconColor.RED) {
//                        return redRedState;
//                    } else if (rightColor == BeaconColorResult.BeaconColor.BLUE) {
//                        return redBlueState;
//                    } else {
//                        return redUnknownState;
//                    }
//                } else if (leftColor == BeaconColorResult.BeaconColor.BLUE) {
//                    if (rightColor == BeaconColorResult.BeaconColor.RED) {
//                        return blueRedState;
//                    } else if (rightColor == BeaconColorResult.BeaconColor.BLUE) {
//                        return blueBlueState;
//                    } else {
//                        return blueUnknownState;
//                    }
//                } else {
//                    if (rightColor == BeaconColorResult.BeaconColor.RED) {
//                        return unknownRedState;
//                    } else if (rightColor == BeaconColorResult.BeaconColor.BLUE) {
//                        return unknownBlueState;
//                    } else {
//                        return unknownUnknownState;
//                    }
//                }
//            }
//        };
//    }

    /**
     * @param nMotors the motors to turn off
     * @return the created State
     */
    public static State stop(final NMotors nMotors) {
        return new BasicAbstractState() {
            @Override
            public void init() {
                nMotors.stop();
            }

            @Override
            public boolean isDone() {
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return null;
            }
        };
    }

    /**
     * @param mecanumControl the motors to turn off
     * @return the created State
     */
    public static State stop(final MecanumControl mecanumControl) {
        return new BasicAbstractState() {
            @Override
            public void init() {
                mecanumControl.stop();
            }

            @Override
            public boolean isDone() {
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return null;
            }
        };
    }

    /**
     * @param message the message to display to the driver station
     * @param value   the value associated with that message
     * @return the created State
     */
    public static State telemetry(final String message, final double value) {
        return new BasicAbstractState() {
            @Override
            public void init() {
            }

            @Override
            public boolean isDone() {
                telemetry.addData(message, value);
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return null;
            }
        };
    }

    /**
     * @param message1 the first message to display to the driver station
     * @param input1   an InputExtractor that returns the value associated with the first message
     * @param message2 the second message to display to the driver station
     * @param input2   an InputExtractor that returns the value associated with the second message
     * @return the created State
     */
    public static State telemetry(final String message1, final InputExtractor<Double> input1, final String message2, final InputExtractor<Double> input2) {
        return new BasicAbstractState() {
            @Override
            public void init() {
            }

            @Override
            public boolean isDone() {
                telemetry.addData(message1, input1.getValue());
                telemetry.addData(message2, input2.getValue());
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return null;
            }
        };
    }

    /**
     * @param transitions the list of transitions to the next states
     * @param message     the message to display to the driver station
     * @param value       the value associated with that message
     * @return the created State
     */
    public static State telemetry(Map<StateName, EndCondition> transitions, final String message, final double value) {
        return new AbstractState(transitions) {
            @Override
            public void init() {
            }

            @Override
            public void run() {
                telemetry.addData(message, value);
            }

            @Override
            public void dispose() {
            }
        };
    }

    /**
     * @param transitions the list of transitions to the next states
     * @param message1    the first message to display to the driver station
     * @param input1      an InputExtractor that returns the value associated with the first message
     * @param message2    the second message to display to the driver station
     * @param input2      an InputExtractor that returns the value associated with the second message
     * @return the created State
     */
    public static State telemetry(Map<StateName, EndCondition> transitions, final String message1, final InputExtractor<Double> input1, final String message2, final InputExtractor<Double> input2) {
        return new AbstractState(transitions) {
            @Override
            public void init() {
            }

            @Override
            public void run() {
                telemetry.addData(message1, input1.getValue());
                telemetry.addData(message2, input2.getValue());
            }

            @Override
            public void dispose() {
            }
        };
    }

    /**
     * @param nextStateName the name of the next state
     * @param servos        the servos to be initialized
     * @return the created State
     * @see Servos
     */
    public static State servoInit(final StateName nextStateName, final Servos servos) {
        return new BasicAbstractState() {
            @Override
            public void init() {
            }

            @Override
            public boolean isDone() {
                return servos.areDone();
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }

    /**
     * @param nextStateName the name of the next state
     * @param gyro          the gyro sensor to be calibrated
     * @return the created State
     * @see Gyro
     */
    public static State calibrateGyro(final StateName nextStateName, final Gyro gyro) {
        return new BasicAbstractState() {
            @Override
            public void init() {
            }

            @Override
            public boolean isDone() {
                return !gyro.isCalibrating();
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }

    /**
     * @param nextStateName    the name of the next state
     * @param doubleLineSensor the 2 line sensors to be calibrated
     * @return the created State
     * @see DoubleLineSensor
     */
    public static State calibrateLineSensor(final StateName nextStateName, final DoubleLineSensor doubleLineSensor) {
        doubleLineSensor.calibrate();
        return new BasicAbstractState() {
            @Override
            public void init() {
            }

            @Override
            public boolean isDone() {
                return doubleLineSensor.isReady();
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }

    /**
     * Turn a servo to a preset at max speed
     *
     * @param nextStateName the name of the state to go to next
     * @param servoControl  the servo
     * @param servoPreset   the preset to go to
     * @param waitForDone   whether to wait for the servo to finish turning or move to the next state immediately
     * @return the created State
     * @see ServoControl
     */
    public static State servoTurn(StateName nextStateName, ServoControl servoControl, Enum servoPreset, boolean waitForDone) {
        return servoTurn(nextStateName, servoControl, servoPreset, ServoControl.MAX_SPEED, waitForDone);
    }



    /**
     * Turn a servo to a preset at a given speed
     *
     * @param nextStateName the name of the state to go to next
     * @param servoControl  the servo
     * @param servoPreset   the preset to go to
     * @param speed         the speed to turn the servo at
     * @param waitForDone   whether to wait for the servo to finish turning or move to the next state immediately
     * @return the created State
     * @see ServoControl
     */
    public static State servoTurn(final StateName nextStateName, final ServoControl servoControl, final Enum servoPreset, final double speed, final boolean waitForDone) {
        return new BasicAbstractState() {

            @Override
            public void init() {
                servoControl.goToPreset(servoPreset, speed);
            }

            @Override
            public boolean isDone() {
                return !waitForDone || servoControl.isDone();
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }

    public static State servoTurn(final StateName nextStateName, final ServoControl servoControl, final Enum servoPreset, final long milliseconds, final boolean waitForDone) {
        return new BasicAbstractState() {
            long intial;
            boolean isDone=false;


            @Override
            public void init() {

                intial= System.currentTimeMillis();

                servoControl.goToPreset(servoPreset);
            }

            @Override
            public boolean isDone() {
                if((intial+milliseconds)>= System.currentTimeMillis()){
                    isDone=true;
                }


                return (!waitForDone || servoControl.isDone())&& isDone;
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }

    /**
     * Turn a servo to a position at max speed
     *
     * @param nextStateName the name of the state to go to next
     * @param servoControl  the servo
     * @param servoPosition the position to go to
     * @param waitForDone   whether to wait for the servo to finish turning or move to the next state immediately
     * @return the created State
     * @see ServoControl
     */
    public static State servoTurn(StateName nextStateName, ServoControl servoControl, double servoPosition, boolean waitForDone) {
        return servoTurn(nextStateName, servoControl, servoPosition, ServoControl.MAX_SPEED, waitForDone);
    }

    /**
     * Turn a servo to a position at a given speed
     *
     * @param nextStateName the name of the state to go to next
     * @param servoControl  the servo
     * @param servoPosition the position to go to
     * @param speed         the speed to turn the servo at
     * @param waitForDone   whether to wait for the servo to finish turning or move to the next state immediately
     * @return the created State
     * @see ServoControl
     */
    public static State servoTurn(final StateName nextStateName, final ServoControl servoControl, final double servoPosition, final double speed, final boolean waitForDone) {
        return new BasicAbstractState() {

            @Override
            public void init() {
                servoControl.setPosition(servoPosition, speed);
            }

            @Override
            public boolean isDone() {
                return !waitForDone || servoControl.isDone();
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }


    /**
     * drive using the mecanum wheels
     * travels for a certain amount of time defined by the robots speed and a desired distance
     *
     * @param nextStateName   the next state to go to
     * @param distance        the distance to travel
     * @param mecanumControl  the mecanum wheels
     * @param gyro            the gyro sensor
     * @param velocity        the velocity to drive at
     * @param direction       the direction to drive
     * @param orientation     the angle to rotate to
     * @param maxAngularSpeed the max speed to rotate to that angle
     * @return the created State
     * @see MecanumControl
     * @see Gyro
     * @see Distance
     */
    public static State mecanumDrive(final StateName nextStateName, final Distance distance, final MecanumControl mecanumControl, final Gyro gyro, final double gyroGain, final double velocity, final Angle direction, final Angle orientation, final Angle tolerance, final double maxAngularSpeed) {
        return mecanumDrive(nextStateName, distance,gyro, mecanumControl,
                RotationControls.gyro(gyro, gyroGain, orientation, tolerance, maxAngularSpeed),
                TranslationControls.constant(velocity, direction)
        );
    }

    public static State mecanumDrive(final StateName nextStateName, final Distance distance, final MecanumControl mecanumControl, final Gyro gyro, final double gyroGain, Vector2D vector2D, final Angle orientation, final Angle tolerance, final double maxAngularSpeed) {
        return mecanumDrive(nextStateName, distance,gyro, mecanumControl,
                RotationControls.gyro(gyro, gyroGain, orientation, tolerance, maxAngularSpeed),
                TranslationControls.constant(vector2D)
        );
    }
//
//    public static State mecanumDrive(StateName nextStateName, Distance distance, final MecanumControl mecanumControl, final Gyro gyro, Vector2D vector2D, final Angle orientation, final Angle tolerance) {
//        return mecanumDrive(nextStateName, distance, mecanumControl, gyro, vector2D, orientation, tolerance, RotationControl.DEFAULT_MAX_ANGULAR_SPEED);
//    }

    public static State mecanumDrive(final StateName nextStateName, final Distance distance, final Gyro gyro, final MecanumControl mecanumControl, final RotationControl rotationControl, final TranslationControl translationControl) {
        mecanumControl.setDriveMode(MecanumMotors.MecanumDriveMode.NORMALIZED);
//        double speedMetersPerMillisecond = mecanumControl.getMaxRobotSpeed(Angle.subtract(direction, orientation)).metersPerMillisecond() * velocity;
//        double durationMillis = Math.abs(distance.meters() / speedMetersPerMillisecond);
//        final EndCondition gyroEC = EVEndConditions.gyroCloseTo(gyro, orientation, tolerance);
//        final EndCondition timeEC = EVEndConditions.timed((long) durationMillis);

        return new BasicAbstractState() {
            Distance distanceTravelled = Distance.zero();
            long lastTime = 0;

            @Override
            public void init() {
                distanceTravelled = Distance.zero();
                lastTime = System.currentTimeMillis();
//                timeEC.init();
//                gyroEC.init();
                mecanumControl.setControl(rotationControl, translationControl);
                gyro.setActive(true);


            }

            @Override
            public boolean isDone() {
                long now = System.currentTimeMillis();
                Time deltaTime = Time.fromMilliseconds(lastTime - now);
                lastTime = now;

//                Log.i("Drive", "distanceTravelled: " + distanceTravelled);
//                Log.i("Drive", "delta distance: " + Distance.multiply(
//                        mecanumControl.getMaxRobotSpeed(Angle.subtract(direction, Angle.fromDegrees(gyro.getHeading()))).getDistance(deltaTime),
//                        velocity * mecanumControl.getMecanumMotors().getScaleFactor()
//                ));
//                Log.i("Drive", "maxRobotSpeed: " + mecanumControl.getMaxRobotSpeed(Angle.subtract(direction, Angle.fromDegrees(gyro.getHeading()))));
//                Log.i("Drive", "velocity * scaleFactor: " + velocity * mecanumControl.getMecanumMotors().getScaleFactor());
//                Log.i("Drive", "drive direction: " + Angle.subtract(direction, Angle.fromDegrees(gyro.getHeading())));
//
                Vector2D translation = new Vector2D(
                        mecanumControl.getVelocityX(),
                        mecanumControl.getVelocityY()
                );

                distanceTravelled = Distance.add(
                        distanceTravelled,

                        Distance.multiply(
                                mecanumControl.getMaxRobotSpeed(translation.getDirection()).getDistance(deltaTime),
//                                mecanumControl.getMaxRobotSpeed(Angle.subtract(translation.getDirection(), Angle.fromDegrees(gyro.getHeading()))).getDistance(deltaTime),
                                translation.getLength() * mecanumControl.getMecanumMotors().getScaleFactor()
                        ).abs()
                );
                return distanceTravelled.meters() >= distance.meters();
//                return timeEC.isDone();
//                if (timeEC.isDone()) {
//                    mecanumControl.setTranslationControl(TranslationControls.ZERO);
//                    return gyroEC.isDone();
//                }
//                return false;
            }

            @Override
            public StateName getNextStateName() {
                gyro.setActive(false);
                mecanumControl.stop();
                return nextStateName;
            }
        };
    }

    public static State mecanumDriveWithSensor(final StateName nextStateName, final Distance bailDistance,
                                               final InputExtractor<Double> sensorReading, final double sensorTolerance, final double gain,
                                               final Gyro gyro,
                                               final double gyroGain, final MecanumControl mecanumControl,
                                               final Vector2D vector2D, final double minVelocity,
                                               final Angle orientation, Angle angleTolerance, final double maxAngularSpeed) {

        TranslationControl transCtrl = new TranslationControl() {
            @Override
            public boolean act() {
                return true;
            }

            @Override
            public Vector2D getTranslation() {
                double sensorValue = sensorReading.getValue();

                double velocityScale = sensorValue*gain;

                Vector2D v = vector2D;
                if(velocityScale<1.0){
                    double x = vector2D.getX()*velocityScale;
                    double y = vector2D.getY()*velocityScale;
                    v = new Vector2D(x, y);
                    double velocityMagnitude = v.getLength();
                    if (velocityMagnitude<minVelocity) {
                        double f = minVelocity / velocityMagnitude;
                        v = new Vector2D(x*f, y*f);
                    }
                }
                return v;
            }
        };

        RotationControl rotationControl = RotationControls.gyro(gyro, gyroGain, orientation, angleTolerance, maxAngularSpeed);


    return mecanumDriveWithSensor2(nextStateName, bailDistance, sensorReading, sensorTolerance,gyro,mecanumControl, rotationControl, transCtrl );

    }

    public static State mecanumDriveWithSensor2(final StateName nextStateName, final Distance bailDistance, final InputExtractor<Double> sensorValue, final double tolerance, final Gyro gyro, final MecanumControl mecanumControl, final RotationControl rotationControl, final TranslationControl translationControl) {
        mecanumControl.setDriveMode(MecanumMotors.MecanumDriveMode.NORMALIZED);
//        double speedMetersPerMillisecond = mecanumControl.getMaxRobotSpeed(Angle.subtract(direction, orientation)).metersPerMillisecond() * velocity;
//        double durationMillis = Math.abs(distance.meters() / speedMetersPerMillisecond);
//        final EndCondition gyroEC = EVEndConditions.gyroCloseTo(gyro, orientation, tolerance);
//        final EndCondition timeEC = EVEndConditions.timed((long) durationMillis);

        return new BasicAbstractState() {
            Distance distanceTravelled = Distance.zero();
            long lastTime = 0;

            @Override
            public void init() {
                distanceTravelled = Distance.zero();
                lastTime = System.currentTimeMillis();
//                timeEC.init();
//                gyroEC.init();
                mecanumControl.setControl(rotationControl, translationControl);
                gyro.setActive(true);


            }

            @Override
            public boolean isDone() {
                long now = System.currentTimeMillis();
                Time deltaTime = Time.fromMilliseconds(lastTime - now);
                lastTime = now;

//                Log.i("Drive", "distanceTravelled: " + distanceTravelled);
//                Log.i("Drive", "delta distance: " + Distance.multiply(
//                        mecanumControl.getMaxRobotSpeed(Angle.subtract(direction, Angle.fromDegrees(gyro.getHeading()))).getDistance(deltaTime),
//                        velocity * mecanumControl.getMecanumMotors().getScaleFactor()
//                ));
//                Log.i("Drive", "maxRobotSpeed: " + mecanumControl.getMaxRobotSpeed(Angle.subtract(direction, Angle.fromDegrees(gyro.getHeading()))));
//                Log.i("Drive", "velocity * scaleFactor: " + velocity * mecanumControl.getMecanumMotors().getScaleFactor());
//                Log.i("Drive", "drive direction: " + Angle.subtract(direction, Angle.fromDegrees(gyro.getHeading())));
//
                Vector2D translation = new Vector2D(
                        mecanumControl.getVelocityX(),
                        mecanumControl.getVelocityY()
                );

                distanceTravelled = Distance.add(
                        distanceTravelled,
                        Distance.multiply(
                                mecanumControl.getMaxRobotSpeed(translation.getDirection()).getDistance(deltaTime),
//                                mecanumControl.getMaxRobotSpeed(Angle.subtract(translation.getDirection(), Angle.fromDegrees(gyro.getHeading()))).getDistance(deltaTime),
                                translation.getLength() * mecanumControl.getMecanumMotors().getScaleFactor()
                        ).abs()
                );
//                if( Math.abs(sensorValue.getValue()) <= tolerance) {
////                    LogPoints.stateReason = "sensor value";
//                } distanceTravelled.meters() >= bailDistance.meters();
                return Math.abs(sensorValue.getValue()) <= tolerance || distanceTravelled.meters() >= bailDistance.meters();

//                return timeEC.isDone();
//                if (timeEC.isDone()) {
//                    mecanumControl.setTranslationControl(TranslationControls.ZERO);
//                    return gyroEC.isDone();
//                }
//                return false;
            }

            @Override
            public StateName getNextStateName() {
                gyro.setActive(false);
                mecanumControl.stop();
                return nextStateName;
            }
        };
    }

    /**
     * drive using the mecanum wheels
     *
     * @param transitions     the list of transitions to the next states
     * @param mecanumControl  the mecanum wheels
     * @param gyro            the gyro sensor
     * @param velocity        the velocity to drive at
     * @param direction       the direction to drive
     * @param orientation     the angle to rotate to
     * @param maxAngularSpeed the max speed to rotate to that angle
     * @return the created State
     * @see MecanumControl
     * @see Gyro
     */
    public static State mecanumDrive(Map<StateName, EndCondition> transitions, final MecanumControl mecanumControl, final Gyro gyro, final double gyroGain, final double velocity, final Angle direction, final Angle orientation, final Angle tolerance, final double maxAngularSpeed) {
//        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile("AutoOptions.txt"));
//
//        double max = optionsFile.get("gyro_max", Double.class);
//        double gain = optionsFile.get("gyro_gain", Double.class);
//
//        RotationControls.gyro(gyro, orientation, max, false, gain)

        return mecanumDrive(transitions, mecanumControl,
                RotationControls.gyro(gyro, gyroGain, orientation, tolerance, maxAngularSpeed),
                TranslationControls.constant(velocity, direction)
        );
    }


    public static State mecanumDrive(Map<StateName, EndCondition> transitions, final MecanumControl mecanumControl, final Gyro gyro, final double gyroGain, Vector2D vector2D, final Angle orientation, final Angle tolerance, final double maxAngularSpeed) {
        return mecanumDrive(transitions, mecanumControl,
                RotationControls.gyro(gyro, gyroGain, orientation, tolerance, maxAngularSpeed),
                TranslationControls.constant(vector2D)
        );
    }

    public static State mecanumDrive(Map<StateName, EndCondition> transitions, MecanumControl mecanumControl, Gyro gyro, final double gyroGain, Vector2D vector2D, Angle orientation, final Angle tolerance) {
        return mecanumDrive(transitions, mecanumControl, gyro, gyroGain, vector2D, orientation, tolerance, RotationControl.DEFAULT_MAX_ANGULAR_SPEED);
    }

    public static State mecanumDrive(Map<StateName, EndCondition> transitions, final MecanumControl mecanumControl, final RotationControl rotationControl, final TranslationControl translationControl) {
        mecanumControl.setDriveMode(MecanumMotors.MecanumDriveMode.NORMALIZED);
        return new AbstractState(transitions) {
            @Override
            public void init() {
                mecanumControl.setControl(rotationControl, translationControl);
            }

            @Override
            public void run() {

            }

            @Override
            public void dispose() {
                mecanumControl.stop();
            }
        };
    }

    /**
     * @param transitions    the list of transitions to the next states
     * @param mecanumControl the mecanum wheels
     * @param gyro           the gyro sensor
     * @param velocity       the velocity to drive at
     * @param direction      the direction to drive
     * @param orientation    the angle to rotate to
     * @return the created State
     * @see MecanumControl
     * @see Gyro
     */
    public static State mecanumDrive(Map<StateName, EndCondition> transitions, MecanumControl mecanumControl, Gyro gyro, double gyroGain, double velocity, Angle direction, Angle orientation, final Angle tolerance) {
        return mecanumDrive(transitions, mecanumControl, gyro, gyroGain, velocity, direction, orientation, tolerance, RotationControl.DEFAULT_MAX_ANGULAR_SPEED);
    }



    /**
     * follow a line with the mecanum wheels
     *
     * @param transitions         the list of transitions to the next states
     * @param lostLineState       what state to go to if the line is lost
     * @param mecanumControl      the mecanum wheels
     * @param doubleLineSensor    the 2 line sensors
     * @param velocity            the velocity to drive at
     * @param lineFollowDirection the direction (left or right) to follow the line at
     * @return the created State
     * @see MecanumControl
     * @see TranslationControls
     */
    public State mecanumLineFollow(Map<StateName, EndCondition> transitions, StateName lostLineState, final MecanumControl mecanumControl, final DoubleLineSensor doubleLineSensor, final double velocity, final TranslationControls.LineFollowDirection lineFollowDirection) {
        transitions.put(lostLineState, new EndCondition() {
            @Override
            public void init() {
            }

            @Override
            public boolean isDone() {
                return !mecanumControl.translationWorked();
            }
        });

        mecanumControl.setDriveMode(MecanumMotors.MecanumDriveMode.NORMALIZED);

        return new AbstractState(transitions) {
            @Override
            public void init() {
                mecanumControl.setTranslationControl(TranslationControls.lineFollow(doubleLineSensor, lineFollowDirection, velocity));
            }

            @Override
            public void run() {

            }

            @Override
            public void dispose() {
                mecanumControl.stop();
            }
        };
    }


    /**
     * Drive forward or backward with two motors
     *
     * @param transitions the transitions to new states
     * @param twoMotors   the motors to move
     * @param velocity    the velocity to drive at (negative for backwards)
     * @return the created State
     * @see TwoMotors
     */
    public static State drive(Map<StateName, EndCondition> transitions, final TwoMotors twoMotors, final double velocity) {
        return new AbstractState(transitions) {
            @Override
            public void init() {
                twoMotors.runMotors(velocity, velocity);
            }

            @Override
            public void run() {

            }

            @Override
            public void dispose() {
                twoMotors.runMotors(0, 0);
            }
        };
    }

    /**
     * Turn left or right
     *
     * @param transitions the transitions to new states
     * @param twoMotors   the motors to move
     * @param velocity    the velocity to turn at (negative for turning left)
     * @return the created State
     * @see TwoMotors
     */
    public static State turn(Map<StateName, EndCondition> transitions, final TwoMotors twoMotors, final double velocity) {
        return new AbstractState(transitions) {
            @Override
            public void init() {
                twoMotors.runMotors(velocity, -velocity);
            }

            @Override
            public void run() {

            }

            @Override
            public void dispose() {
                twoMotors.runMotors(0, 0);
            }
        };
    }

    /**
     * Turn with one wheel
     *
     * @param transitions  the transitions to new states
     * @param twoMotors    the motors to move
     * @param isRightWheel tells which wheel to turn
     * @param velocity     the velocity to turn the wheel at (negative for backwards)
     * @return the created State
     * @see TwoMotors
     */
    public static State oneWheelTurn(Map<StateName, EndCondition> transitions, final TwoMotors twoMotors, final boolean isRightWheel, final double velocity) {
        return new AbstractState(transitions) {
            @Override
            public void init() {
                if (isRightWheel) {
                    twoMotors.runMotors(0, velocity);
                } else {
                    twoMotors.runMotors(velocity, 0);
                }
            }

            @Override
            public void run() {

            }

            @Override
            public void dispose() {
                twoMotors.runMotors(0, 0);
            }
        };
    }


    /**
     * Drive for a certain distance
     *
     * @param nextStateName the state to go to after the drive is done
     * @param distance      the distance to drive
     * @param twoMotors     the motors to move
     * @param velocity      the velocity to drive at (negative for backwards)
     * @return the created State
     * @see TwoMotors
     */
    public static State drive(StateName nextStateName, Distance distance, TwoMotors twoMotors, double velocity) {
        double speedMetersPerMillisecond = twoMotors.getMaxRobotSpeed().metersPerMillisecond() * velocity;
        double durationMillis = Math.abs(distance.meters() / speedMetersPerMillisecond);
        return drive(StateMap.of(
                nextStateName,
                EndConditions.timed((long) durationMillis)
        ), twoMotors, velocity);
    }

    /**
     * Turn for a certain angle by calculating the time required for that angle
     *
     * @param nextStateName    the state to go to when done turning
     * @param angle            the angle to turn
     * @param minRobotTurnTime the time it takes for the robot to turn
     * @param twoMotors        the motors to run
     * @param velocity         the velocity to turn (negative for turning left)
     * @return the created State
     * @see TwoMotors
     */
    public static State turn(StateName nextStateName, Angle angle, Time minRobotTurnTime, TwoMotors twoMotors, double velocity) {
        double speedRotationsPerMillisecond = velocity / minRobotTurnTime.milliseconds();
        double durationMillis = Math.abs(angle.degrees() / 360 / speedRotationsPerMillisecond);
        return turn(StateMap.of(
                nextStateName,
                EndConditions.timed((long) durationMillis)
        ), twoMotors, velocity);
    }

    /**
     * Turn for a certain angle using a gyro sensor
     *
     * @param nextStateName the state to go to when done turning
     * @param angle         the angle to turn
     * @param gyro          the gyro sensor to use
     * @param twoMotors     the motors to turn
     * @param velocity      the velocity to turn at (negative to turn left)
     * @return the created State
     * @see TwoMotors
     */
    public static State turn(StateName nextStateName, Angle angle, Gyro gyro, TwoMotors twoMotors, double velocity) {
        return turn(StateMap.of(
                nextStateName,
                EVEndConditions.gyroCloseToRelative(gyro, angle, Angle.fromDegrees(5))
        ), twoMotors, velocity);
    }

    /**
     * Turn with one wheel for a certain angle by calculating the time needed to turn that angle
     *
     * @param nextStateName    the state to go to when done turning
     * @param angle            the angle to turn
     * @param minRobotTurnTime the time it takes for the robot to turn
     * @param twoMotors        the motors to turn
     * @param isRightWheel     tells which wheel to turn
     * @param velocity         the velocity to turn the wheel at (negative for backwards)
     * @return the created State
     * @see TwoMotors
     */
    public static State oneWheelTurn(StateName nextStateName, Angle angle, Time minRobotTurnTime, TwoMotors twoMotors, boolean isRightWheel, double velocity) {
        double speedRotationsPerMillisecond = Math.abs(velocity) / minRobotTurnTime.milliseconds();
        double durationMillis = Math.abs(2 * angle.degrees() / 360 / speedRotationsPerMillisecond);
        double velocity1 = Math.abs(velocity) * Math.signum(angle.radians());
        if (isRightWheel) {
            velocity1 *= -1;
        }
        return oneWheelTurn(StateMap.of(
                nextStateName,
                EndConditions.timed((long) durationMillis)
        ), twoMotors, isRightWheel, velocity1);
    }

    /**
     * Turn with one wheel for a certain angle using a gyro sensor
     *
     * @param nextStateName the state to go to after the turn is done
     * @param angle         the angle to turn
     * @param gyro          the gyro sensor to use
     * @param twoMotors     the motors to turn
     * @param isRightWheel  which wheel to use
     * @param velocity      the velocity to turn the wheel at (negative for backwards)
     * @return the created State
     * @see TwoMotors
     */
    public static State turn(StateName nextStateName, Angle angle, Gyro gyro, TwoMotors twoMotors, boolean isRightWheel, double velocity) {
        double velocity1 = Math.abs(velocity) * Math.signum(angle.radians());
        if (isRightWheel) {
            velocity1 *= -1;
        }
        return oneWheelTurn(StateMap.of(
                nextStateName,
                EVEndConditions.gyroCloseToRelative(gyro, angle, Angle.fromDegrees(5))
        ), twoMotors, isRightWheel, velocity1);
    }

    /**
     * Turn a motor at a given power
     *
     * @param transitions the transitions to new states
     * @param motor       the motor to be turned
     * @param power       the power to turn the motor at
     * @return the created State
     * @see TwoMotors
     */
    public static State motorTurn(Map<StateName, EndCondition> transitions, final Motor motor, final double power, final boolean useSpeedMode) {
        final MotorEnc motorEnc = useSpeedMode ? (MotorEnc) motor : null;

        return new AbstractState(transitions) {
            @Override
            public void init() {
                if (useSpeedMode) {
                    motorEnc.setSpeed(power);
                } else {
                    motor.setPower(power);
                }
            }

            @Override
            public void run() {

            }

            @Override
            public void dispose() {
                if (useSpeedMode) {
                    motorEnc.setSpeed(0);
                } else {
                    motor.setPower(0);
                }
            }
        };
    }

    /**
     * Turn a motor on and immediately move to the next state
     *
     * @param nextStateName the state to go to next
     * @param motor         the motor to turn on
     * @param power         the power to run the motor at
     * @return the created State
     */
    public static State motorOn(final StateName nextStateName, final Motor motor, final double power) {
        return new State() {
            @Override
            public StateName act() {
                motor.setPower(power);
                return nextStateName;
            }
        };
    }

    /**
     * Turn a motor off and immediately move to the next state
     *
     * @param nextStateName the state to go to next
     * @param motor         the motor to turn off
     * @return the created State
     */
    public static State motorOff(final StateName nextStateName, final Motor motor) {
        return new State() {
            @Override
            public StateName act() {
//                if (motor.getMode() == Motor.Mode.SPEED) {
//                    ((MotorEnc) motor).setSpeed(0);
//                } else {
                motor.setPower(0);
//                }
                return nextStateName;
            }
        };
    }

//    /**
//     * Line up with the beacon using the line sensor array and distance sensor
//     *
//     * @param successState      the state to go to if the line up succeeds
//     * @param failState         the state to go to if the line up fails
//     * @param mecanumControl    the mecanum wheels
//     * @param direction         the direction angle to face
//     * @param gyro              the gyro to use for rotation stabilization
//     * @param distSensor        the distance sensor to detect distance from the beacon
//     * @param lineSensorArray   the line sensor array to line up sideways with the line
//     * @param teamColor         the team you are on and ...
//     * @param beaconColorResult ... the beacon configuration to decide which button to line up with
//     * @param distance          the distance from the beacon to line up to
//     * @return the created State
//     * @see LineUpControl
//     */
//    public static State beaconLineUp(final StateName successState, final StateName failState, final MecanumControl mecanumControl, final Angle direction, final Gyro gyro, final DistanceSensor distSensor, final LineSensorArray lineSensorArray, TeamColor teamColor, final ResultReceiver<BeaconColorResult> beaconColorResult, final Distance distance) {
////        final EndCondition distEndCondition = EVEndConditions.distanceSensorLess(distSensor, Distance.add(distance, Distance.fromInches(4)));
//        final EndCondition distEndCondition = EVEndConditions.distanceSensorLess(distSensor, distance);
//        final EndCondition gyroEndCondition = EVEndConditions.gyroCloseTo(gyro, direction, 2);
//
//        final BeaconColorResult.BeaconColor myColor = BeaconColorResult.BeaconColor.fromTeamColor(teamColor);
//        final BeaconColorResult.BeaconColor opponentColor = BeaconColorResult.BeaconColor.fromTeamColor(teamColor.opposite());
//
//        return new BasicAbstractState() {
//            private boolean success;
//            LineUpControl.Button buttonToLineUpWith;
//
//            @Override
//            public void init() {
//                buttonToLineUpWith = null;
//                if (beaconColorResult.isReady()) {
//                    BeaconColorResult result = beaconColorResult.getValue();
//                    BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
//                    BeaconColorResult.BeaconColor rightColor = result.getRightColor();
//                    if (Objects.equals(leftColor, myColor) && Objects.equals(rightColor, opponentColor)) {
//                        buttonToLineUpWith = LineUpControl.Button.LEFT;
//                    }
//                    if (Objects.equals(leftColor, opponentColor) && Objects.equals(rightColor, myColor)) {
//                        buttonToLineUpWith = LineUpControl.Button.RIGHT;
//                    }
//                }
//
//                success = buttonToLineUpWith != null;
//
//                LineUpControl lineUpControl = new LineUpControl(lineSensorArray, buttonToLineUpWith, distSensor, distance, gyro, direction);
//
//                mecanumControl.setTranslationControl(lineUpControl);
//                mecanumControl.setRotationControl(lineUpControl);
//
//                distEndCondition.init();
//                gyroEndCondition.init();
//            }
//
//            @Override
//            public boolean isDone() {
//
//                if (!mecanumControl.translationWorked()) {
//                    success = false;
//                }
//                return !success || distEndCondition.isDone();
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                mecanumControl.stop();
//                return success ? successState : failState;
//            }
//        };}


    public static State gyroTurn(StateName nextStateName, final MecanumControl mecanumControl,
                                 final Gyro gyro, final double gyroGain, final Angle orientation, final Angle tolerance) {
    return gyroTurn(nextStateName,mecanumControl,gyro, gyroGain, orientation,tolerance,1);
    }

    public static State gyroTurn(StateName nextStateName, final MecanumControl mecanumControl,
                                 final Gyro gyro, final double gyroGain, final Angle orientation, final Angle tolerance, final double speed) {
        Map<StateName, EndCondition> transitions = StateMap.of(
                nextStateName,
                EVEndConditions.gyroCloseTo(gyro, orientation, tolerance)
        );

        return new AbstractState(transitions) {
            @Override
            public void init() {
                mecanumControl.setTranslationControl(TranslationControls.ZERO);
                mecanumControl.setRotationControl(RotationControls.gyro(gyro, gyroGain, orientation, tolerance, speed));
                gyro.setActive(true);
            }

            @Override
            public void run() {

            }

            @Override
            public void dispose() {
                mecanumControl.stop();
                gyro.setActive(false);
            }
        };
    }

    public static State switchPressed(StateName pressedStateName, StateName timeoutStateName, DigitalSensor digitalSensor, Time timeout) {
        return States.empty(StateMap.of(
                pressedStateName, EndConditions.inputExtractor(digitalSensor),
                timeoutStateName, EndConditions.timed((long) timeout.milliseconds())
        ));
    }

//    public static State beaconColorSwitch(final StateName leftButtonState, final StateName rightButtonState, final StateName unknownState, TeamColor teamColor, final ResultReceiver<BeaconColorResult> beaconColorResult) {
//
//        final BeaconColorResult.BeaconColor myColor = BeaconColorResult.BeaconColor.fromTeamColor(teamColor);
//        final BeaconColorResult.BeaconColor opponentColor = BeaconColorResult.BeaconColor.fromTeamColor(teamColor.opposite());
//
//        return new State() {
//
//            @Override
//            public StateName act() {
//                if (beaconColorResult.isReady()) {
//                    BeaconColorResult result = beaconColorResult.getValue();
//                    BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
//                    BeaconColorResult.BeaconColor rightColor = result.getRightColor();
//                    if (Objects.equals(leftColor, myColor) && Objects.equals(rightColor, opponentColor)) {
//                        return leftButtonState;
//                    }
//                    if (Objects.equals(leftColor, opponentColor) && Objects.equals(rightColor, myColor)) {
//                        return rightButtonState;
//                    }
//                }
//
//                return unknownState;
//            }
//        };
//    }

    public static State servoAdd(final StateName nextStateName, final ServoControl servoControl, final Angle angleToAdd, final boolean waitForDone, final Function degreesFromServoPos, final Function servoPosFromDegrees) {
        return new BasicAbstractState() {

            @Override
            public void init() {
                Angle currentAngle = Angle.fromDegrees(degreesFromServoPos.f(servoControl.getCurrentPosition()));
                Angle targetAngle = Angle.add(currentAngle, angleToAdd);
                double targetPosition = servoPosFromDegrees.f(targetAngle.degrees());
                servoControl.setPosition(Utility.servoLimit(targetPosition));
            }

            @Override
            public boolean isDone() {
                return !waitForDone || servoControl.isDone();
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }
    public static State servoAdd(final StateName nextStateName, final ServoControl servoControl, final double numToAdd, final boolean waitForDone) {
        return new BasicAbstractState() {

            @Override
            public void init() {

                servoControl.setPosition(Utility.servoLimit(servoControl.getCurrentPosition()+numToAdd));
            }

            @Override
            public boolean isDone() {
                return !waitForDone || servoControl.isDone();
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }
    public static State servoAdd(final StateName nextStateName, final ServoControl servoControl, final double numToAdd, final double speed, final boolean waitForDone) {
        return new BasicAbstractState() {

            @Override
            public void init() {

                servoControl.setPosition(Utility.servoLimit(servoControl.getCurrentPosition()+numToAdd),speed);
            }

            @Override
            public boolean isDone() {
                return !waitForDone || servoControl.isDone();
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }

    public static State resultReceiverReady(final StateName nextStateName, final ResultReceiver resultReceiver){
        return new BasicAbstractState() {

            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return resultReceiver.isReady();
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }
}
