package org.firstinspires.ftc.teamcode.GameChangersTester;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.AnalogInputEdgeDetector;
import ftc.evlib.hardware.motors.MotorEncEx;

public class ShooterState extends BasicAbstractState {

    private AnalogInputEdgeDetector collectorShooterButton;
    private GameChangersRobotCfg robotCfg;
    private long initTime;
    private long firstPause;
    private double targetSpeed;
    private int numCycles = 0;
    private final StateName nextState;
    private MotorEncEx shooterMotor;
    private boolean released;

    public ShooterState(AnalogInputEdgeDetector collectorShooterButton, GameChangersRobotCfg robotCfg, long firstPause, MotorEncEx shooterMotor, double targetSpeed, StateName next) {
        this.collectorShooterButton = collectorShooterButton;
        this.robotCfg = robotCfg;
        this.firstPause = firstPause;
        this.nextState = next;
        this.targetSpeed = targetSpeed;
        this.shooterMotor = shooterMotor;
    }

    public ShooterState(GameChangersRobotCfg robotCfg, long firstPause, MotorEncEx shooterMotor, double targetSpeed, StateName next) {
        this.collectorShooterButton = null;
        this.robotCfg = robotCfg;
        this.firstPause = firstPause;
        this.nextState = next;
        this.targetSpeed = targetSpeed;
        this.shooterMotor = shooterMotor;
    }

    @Override
    public void init() {
        initTime = System.currentTimeMillis();
        robotCfg.getPusher().goToPreset(ServoPresets.Pusher.PUSH);
        numCycles = 0;
        released = false;
    }

    @Override
    public boolean isDone() {
        if(collectorShooterButton == null) {
            if(numCycles > 2) {
                robotCfg.getPusher().goToPreset(ServoPresets.Pusher.RELEASE);
                return true;
            }
        } else {
            if (!collectorShooterButton.isPressed()) {
                robotCfg.getPusher().goToPreset(ServoPresets.Pusher.RELEASE);
                return true;
            }
        }
        long timeSinceInit = System.currentTimeMillis() - initTime;

        if(timeSinceInit > firstPause) {
            if (!released) {
                released = true;
                robotCfg.getPusher().goToPreset(ServoPresets.Pusher.RELEASE);
            }
            if (shooterMotor.getVelocity() >= targetSpeed) {
                robotCfg.getPusher().goToPreset(ServoPresets.Pusher.PUSH);
                released = false;
                numCycles++;
                initTime = System.currentTimeMillis();
            }
        }
        return false;
    }

    @Override
    public StateName getNextStateName() {
        return nextState;
    }
}
