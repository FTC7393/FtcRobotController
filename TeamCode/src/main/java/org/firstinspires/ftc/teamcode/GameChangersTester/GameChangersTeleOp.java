//package org.firstinspires.ftc.teamcode.GameChangersTester;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import ftc.electronvolts.util.Function;
//import ftc.electronvolts.util.Functions;
//import ftc.electronvolts.util.files.Logger;
//import ftc.evlib.opmodes.AbstractTeleOp;
//@Disabled
//@TeleOp(name = "anvitaTankDriveOp")
//public class GameChangersTeleOp extends AbstractTeleOp<GameChangersRobotCfg> {
//
//
//    @Override
//    protected Function getJoystickScalingFunction() {
//        return Functions.none();
//    }
//
//    @Override
//    protected GameChangersRobotCfg createRobotCfg() {
//        return new GameChangersRobotCfg(hardwareMap);
//    }
//
//    @Override
//    protected Logger createLogger() {
//        return null;
//    }
//
//    @Override
//    protected void setup() {
//
//    }
//
//    @Override
//    protected void setup_act() {
//
//    }
//
//    @Override
//    protected void go() {
//
//    }
//
//    @Override
//    protected void act() {
//        robotCfg.getTwoMotors().runMotors(
//                driver1.left_stick_y.getValue(),
//                driver1.right_stick_y.getValue()
//        );
//
//        telemetry.addData("left stick y values", driver1.left_stick_y.getValue());
//        telemetry.addData("right stick y values", driver1.right_stick_y.getValue());
//
//    }
//
//    @Override
//    protected void end() {
//
//    }
//}
