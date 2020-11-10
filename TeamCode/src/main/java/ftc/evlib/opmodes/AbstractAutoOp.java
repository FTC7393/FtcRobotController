package ftc.evlib.opmodes;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.util.units.Time;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/13/16
 *
 * extends AbstractOp and adds a 30 second timer and a state machine
 *
 * @see ftc.evlib.opmodes.AbstractOp
 * @see StateMachine
 */
public abstract class AbstractAutoOp<Type extends RobotCfg> extends ftc.evlib.opmodes.AbstractOp<Type> {
    protected StateMachine stateMachine;

    /**
     * This is implemented by the autonomous opmode
     * It is called where the setup would have been
     *
     * @return A state machine to be run
     */
    public abstract StateMachine buildStates();

    @Override
    public Time getMatchTime() {
        return Time.fromSeconds(30); //autonomous is 30 seconds
    }

    @Override
    public void setup() {
        stateMachine = buildStates(); //get the state machine from the opmode
    }

    @Override
    public void pre_act() {

    }

    @Override
    public void post_act() {
        stateMachine.act(); //update the state machine
    }
}
