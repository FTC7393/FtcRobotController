package org.firstinspires.ftc.teamcode.GameChangersTester;


import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.units.Angle;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.hardware.servos.Servos;
import ftc.evlib.statemachine.EVStateMachineBuilder;

public class PowerShotStateMachineFactory {



    public static StateMachine create(TeamColor teamColor, Angle tolerance, Gyro gyro, double gyroGain, double maxAngularSpeed,
                                      Servos servos, MecanumControl mecanumControl, final Continuable button) {
        StateName firstState = S.IDLE;
        EVStateMachineBuilder b = new EVStateMachineBuilder(firstState, teamColor, tolerance, gyro, gyroGain, maxAngularSpeed,
                servos, mecanumControl);
        State idleState = new State() {
            @Override
            public StateName act() {
                if (button.doContinue()) {
                    return S.VUFORIA_SEEK;
                }
                return null;
            }
        };

        State vuforiaSeek = createVuforiaSeekState(S.IDLE);

        b.add(firstState, idleState);

        return b.build();
    }

    private static State createVuforiaSeekState(final StateName nextState) {
        return new State() {
            @Override
            public StateName act() {

                return nextState;
            }
        };
    }

    public enum S implements StateName{
        VUFORIA_SEEK, IDLE
    }

}

