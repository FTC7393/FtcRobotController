package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.AnalogInputEdgeDetector;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.opmodes.AbstractTeleOp;
import ftc.evlib.util.ImmutableList;

@TeleOp(name = "GameChangersTeleOP")
public class GameChangersTeleOP extends AbstractTeleOp<GameChangersRobotCfg>  {

    private boolean wobbleGoalGrabberIsUp = true; //this requires the wobble goal collector to initialize by being up
    private boolean pincherIsClosed = true;
    private boolean collectorIsSucking = false;
    private StateMachine shooterStateMachine;

    private AnalogInputEdgeDetector collectorIntakeButton;
    private AnalogInputEdgeDetector collectorShooterButton;

    public GameChangersTeleOP() {

    }

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.eBased(5);
    }

    @Override
    protected GameChangersRobotCfg createRobotCfg() {
        return new GameChangersRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return new Logger("log_", ".csv", ImmutableList.of(
                new Logger.Column("proportional value potentiometer", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getWobbleGoalArm().getPidController().getpTerm();
                    }
                }),
                new Logger.Column("integral value potentiometer", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getWobbleGoalArm().getPidController().getiTerm();
                    }
                }),
                new Logger.Column("derivative value potentiometer", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getWobbleGoalArm().getPidController().getdTerm();
                    }
                }),
                new Logger.Column("input value potentiometer", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getWobbleGoalArm().getPidController().getInput();
                    }
                }),
                new Logger.Column("output value potentiometer", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getWobbleGoalArm().getPidController().getOutput();
                    }
                }),
                new Logger.Column("error value potentiometer", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getWobbleGoalArm().getPidController().getError();
                    }
                }),
                new Logger.Column("target position value potentiometer", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getWobbleGoalArm().getTargetPosition();
                    }
                })
        ));
    }

    @Override
    protected void setup() {
        collectorIntakeButton = new AnalogInputEdgeDetector(driver1.left_trigger, 0.3, 0.7, false);
        collectorShooterButton = new AnalogInputEdgeDetector(driver1.right_trigger, 0.3, 0.7, false);
        
        shooterStateMachine = buildShooterStateMachine();
    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {
        InputExtractor<Double> invertedRightStick = new InputExtractor<Double>() {
            @Override
            public Double getValue() {
                return -driver1.right_stick_x.getValue();
            }
        };
        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(InputExtractors.negative(driver1.left_stick_x), driver1.left_stick_y));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(invertedRightStick));

    }

    @Override
    protected void act() {
        collectorIntakeButton.update();
        collectorShooterButton.update();
        shooterStateMachine.act();

        telemetry.addData("potentiometer values", robotCfg.getPotentiometer().getValue());

        if(driver1.left_bumper.justPressed()) {
            if(wobbleGoalGrabberIsUp) {
                robotCfg.getWobbleGoalArm().moveArmDown();
            } else {
                robotCfg.getWobbleGoalArm().moveArmUp();
            }
            wobbleGoalGrabberIsUp = !wobbleGoalGrabberIsUp;
        }

        if(driver1.right_bumper.justPressed()) {
            if(pincherIsClosed) {
                robotCfg.getPincher().goToPreset(ServoPresets.WobblePincher.OPENED);
            } else {
                robotCfg.getPincher().goToPreset(ServoPresets.WobblePincher.CLOSED);
            }
            pincherIsClosed = !pincherIsClosed;
        }



        if(collectorIntakeButton.justPressed()) {
            if(collectorIsSucking) {
                robotCfg.getCollector().stop();
            } else {
                robotCfg.getCollector().ingest();
            }
            collectorIsSucking = !collectorIsSucking;
        }

    }

    @Override
    protected void end() {

    }

    private StateMachine buildShooterStateMachine() {
        State idleState = new State() {
            @Override
            public StateName act() {
                if (collectorShooterButton.isPressed()) {
                    return S.SHOOTING;
                }
                return null;
            }
        };

        State shootingState = new ShooterState(collectorShooterButton,robotCfg,300L,1000L);

        StateMachineBuilder b = new StateMachineBuilder(S.IDLE);
        b.add(S.IDLE, idleState);
        b.add(S.SHOOTING, shootingState);



        return b.build();
    }
    enum S implements StateName {
        IDLE,
        SHOOTING
    }
}
