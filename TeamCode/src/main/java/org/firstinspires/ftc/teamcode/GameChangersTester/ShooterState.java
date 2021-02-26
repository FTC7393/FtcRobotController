package org.firstinspires.ftc.teamcode.GameChangersTester;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.AnalogInputEdgeDetector;
import ftc.evlib.hardware.motors.MotorEncEx;

public class ShooterState extends BasicAbstractState {

    private AnalogInputEdgeDetector collectorShooterButton;
    private GameChangersRobotCfg robotCfg;
    private long initTime;
    private long firstPause=120L;
    private long retractTime;
    private long secondPause = firstPause;
    private final double targetSpeed;
    private int numCycles = 0;
    private final StateName nextState;
    private MotorEncEx shooterMotor;
    private boolean released;
    private boolean firstPushStarted;
    private final double maxMotorSpeed;


    public ShooterState(AnalogInputEdgeDetector collectorShooterButton, GameChangersRobotCfg robotCfg, double targetSpeed, double maxMotorSpeed, StateName next) {
        this.collectorShooterButton = collectorShooterButton;
        this.robotCfg = robotCfg;
        this.firstPause = firstPause;
        this.nextState = next;
        this.targetSpeed = targetSpeed;
        this.maxMotorSpeed = maxMotorSpeed;
        this.shooterMotor = robotCfg.getFlyWheelShooter().getMotor();
    }

    public ShooterState(GameChangersRobotCfg robotCfg, double targetSpeed, double maxMotorSpeed, StateName next) {
        this.collectorShooterButton = null;
        this.robotCfg = robotCfg;
        this.firstPause = firstPause;
        this.nextState = next;
        this.targetSpeed = targetSpeed;
        this.maxMotorSpeed = maxMotorSpeed;
        this.shooterMotor = robotCfg.getFlyWheelShooter().getMotor();
    }

    @Override
    public void init() {
        //initTime = System.currentTimeMillis();
        //robotCfg.getPusher().goToPreset(ServoPresets.Pusher.PUSH);
        firstPushStarted =false;
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
        if(!firstPushStarted){
            if(shooterMotor.getVelocity() < maxMotorSpeed || GameChangersTeleOP.powershotMode){
                initTime = System.currentTimeMillis();
                robotCfg.getPusher().goToPreset(ServoPresets.Pusher.PUSH);
                firstPushStarted =true;
            }
        }else {
            long timeSinceInit = System.currentTimeMillis() - initTime;

            if (timeSinceInit > firstPause) {
                if (!released) {
                    released = true;
                    robotCfg.getPusher().goToPreset(ServoPresets.Pusher.RELEASE);
                    retractTime = System.currentTimeMillis();

                } else {
                    long timeSinceRetract = System.currentTimeMillis() - retractTime;

                    if (shooterMotor.getVelocity() >= targetSpeed && timeSinceRetract > secondPause ) {
                        robotCfg.getPusher().goToPreset(ServoPresets.Pusher.PUSH);
                        released = false;
                        numCycles++;
                        initTime = System.currentTimeMillis();
                    }
                }
            }
        }
        return false;
    }

    @Override
    public StateName getNextStateName() {
        return nextState;
    }
}
