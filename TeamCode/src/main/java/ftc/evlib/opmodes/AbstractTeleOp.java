package ftc.evlib.opmodes;

import ftc.electronvolts.util.InitButton;
import ftc.evlib.driverstation.GamepadManager;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.units.Time;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/12/16
 *
 * extends AbstractOp and adds gamepad edge detection and scaling, and a 2 minute timer
 *
 * @see ftc.evlib.opmodes.AbstractOp
 * @see GamepadManager
 */
public abstract class AbstractTeleOp<Type extends RobotCfg> extends ftc.evlib.opmodes.AbstractOp<Type> {
    public GamepadManager driver1;
    public GamepadManager driver2;

    /**
     * This is implemented by the teleop opmode
     *
     * @return a Function to scale the joysticks by
     */
    protected abstract Function getJoystickScalingFunction();

    @Override
    public Time getMatchTime() {
        return Time.fromMinutes(2); //teleop is 2 minutes
    }

    @Override
    public void init() {
        //get the scaling function
        Function f = getJoystickScalingFunction();
        if (f == null) {
            //if it is null set it to none
            f = Functions.none();
        }

        //apply the function to the gamepads and store them
        driver1 = new GamepadManager(gamepad1, f);
        driver2 = new GamepadManager(gamepad2, f);
        driver1 = new GamepadManager(gamepad1, f, InitButton.A);
        driver2 = new GamepadManager(gamepad2, f, InitButton.B);
        super.init();
    }

    @Override
    public void pre_act() {
        //update the joystick values
        driver1.update();
        driver2.update();
    }

    @Override
    public void post_act() {

    }
}
