package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractServoTuneOp;

@TeleOp(name = "ServoTuneOp")
public class GameChangersServoTuneOp extends AbstractServoTuneOp
{

    @Override
    protected RobotCfg createRobotCfg() {
        return new GameChangersRobotCfg(hardwareMap);
    }
}
