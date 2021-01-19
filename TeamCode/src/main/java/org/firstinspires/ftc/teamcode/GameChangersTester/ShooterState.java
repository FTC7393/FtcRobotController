package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.robotcore.util.ElapsedTime;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.AnalogInputEdgeDetector;
import ftc.evlib.hardware.config.RobotCfg;

public class ShooterState extends BasicAbstractState {

    private AnalogInputEdgeDetector collectorShooterButton;
    private GameChangersRobotCfg robotCfg;
    private long initTime;
    private long firstPause;
    private long secondPause;

    public ShooterState(AnalogInputEdgeDetector collectorShooterButton, GameChangersRobotCfg robotCfg, long firstPause, long secondPause) {
        this.collectorShooterButton = collectorShooterButton;
        this.robotCfg = robotCfg;
        this.firstPause = firstPause;
        this.secondPause = secondPause;
    }

    @Override
    public void init() {
        initTime = System.currentTimeMillis();
        robotCfg.getPusher().goToPreset(ServoPresets.Pusher.PUSH);

    }

    @Override
    public boolean isDone() {
        if (!collectorShooterButton.isPressed()) {
            robotCfg.getPusher().goToPreset(ServoPresets.Pusher.RELEASE);
            return true;
        }

        long timeSinceInit = System.currentTimeMillis() - initTime;

        if (timeSinceInit > secondPause) {
            robotCfg.getPusher().goToPreset(ServoPresets.Pusher.PUSH);
            initTime = System.currentTimeMillis();
        } else if (timeSinceInit > firstPause) {
            robotCfg.getPusher().goToPreset(ServoPresets.Pusher.RELEASE);
        }
        return false;
    }

    @Override
    public StateName getNextStateName() {
        return GameChangersTeleOP.S.IDLE;
    }
}
