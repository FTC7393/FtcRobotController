package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.hardware.control.RotationControl;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.opmodes.AbstractTeleOp;

@TeleOp(name = "GameChangersTeleOP")
public class GameChangersTeleOP extends AbstractTeleOp<GameChangersRobotCfg>  {

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
        return null;
    }

    @Override
    protected void setup() {

    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(driver1.left_stick_y, driver1.left_stick_x));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(driver1.right_stick_x));

    }

    @Override
    protected void act() {

    }

    @Override
    protected void end() {

    }
}
