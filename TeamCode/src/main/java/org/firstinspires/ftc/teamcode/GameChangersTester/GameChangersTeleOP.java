package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
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
        InputExtractor<Double> invertedRightStick = new InputExtractor<Double>() {
            @Override
            public Double getValue() {
                return -driver1.right_stick_x.getValue();
            }
        };
        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(driver1.left_stick_x,driver1.left_stick_y));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(invertedRightStick));

    }

    @Override
    protected void act() {

    }

    @Override
    protected void end() {

    }
}
