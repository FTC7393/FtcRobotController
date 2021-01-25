package org.firstinspires.ftc.teamcode.GameChangersTester;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.AnalogInputEdgeDetector;

public class ShooterState extends BasicAbstractState {

    private AnalogInputEdgeDetector collectorShooterButton;
    private GameChangersRobotCfg robotCfg;
    private long initTime;
    private long firstPause;
    private long secondPause;
    private int numCycles = 0;
    private final StateName nextState;

    public ShooterState(AnalogInputEdgeDetector collectorShooterButton, GameChangersRobotCfg robotCfg, long firstPause, long secondPause, StateName next) {
        this.collectorShooterButton = collectorShooterButton;
        this.robotCfg = robotCfg;
        this.firstPause = firstPause;
        this.secondPause = secondPause;
        this.nextState = next;
    }

    public ShooterState(GameChangersRobotCfg robotCfg, long firstPause, long secondPause, StateName next) {
        this.collectorShooterButton = null;
        this.robotCfg = robotCfg;
        this.firstPause = firstPause;
        this.secondPause = secondPause;
        this.nextState = next;
    }

    @Override
    public void init() {
        initTime = System.currentTimeMillis();
        robotCfg.getPusher().goToPreset(ServoPresets.Pusher.PUSH);

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

        if (timeSinceInit > secondPause) {
            robotCfg.getPusher().goToPreset(ServoPresets.Pusher.PUSH);
            initTime = System.currentTimeMillis();
            numCycles++;
        } else if (timeSinceInit > firstPause) {
            robotCfg.getPusher().goToPreset(ServoPresets.Pusher.RELEASE);
        }
        return false;
    }

    @Override
    public StateName getNextStateName() {
        return nextState;
    }
}
