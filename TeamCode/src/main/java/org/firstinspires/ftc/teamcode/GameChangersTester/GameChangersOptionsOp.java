package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.Utility;
import ftc.evlib.opmodes.AbstractOptionsOp;

@TeleOp(name = "GameChangers OptionsOp")
public class GameChangersOptionsOp extends AbstractOptionsOp{
    /**
     * Created by ftc7393 on 12/6/2017.
     */
        public static final String FILENAME = "options_gamechangers.txt";
        public static final String teamColorTag = "teamColor";
        public static final String startingPositionTag = "startPosition";
        public static final StartingPosition startingPositionDefault = StartingPosition.LEFT;
        public static final TeamColor teamColorDefault = TeamColor.BLUE;
        public static final String initialAutoDelayTag = "initialAutoDelay";
        private int index = 0;
        private Opts[] values;
        public static final double initialAutoDelayDefault = 0;


    public GameChangersOptionsOp() {
            super(FILENAME);
            values = Opts.values();
        }

    /**
     * The filename will be set by the subclasses
     *
     * @param filename the name of the file where the options are stored
     */
    public GameChangersOptionsOp(String filename) {
        super(filename);
        values = Opts.values();
    }

    @Override
        protected void go() {
            super.go();

        }

        @Override
        protected void act() {

            telemetry.addData("option", values[index]);
            if (driver1.x.justPressed()) {
                index++;
                if (index >= values.length)
                    index = 0;
            }
            if (driver1.b.justPressed()) {
                index--;
                if (index < 0) {
                    index = values.length - 1;
                }
            }

            if (driver1.right_bumper.justPressed()) {
                if (values[index] == Opts.TEAM_COLOR) {
                    TeamColor teamColor = optionsFile.get(Opts.TEAM_COLOR.s, teamColorDefault);
                    if (teamColor == TeamColor.BLUE) {
                        teamColor = TeamColor.RED;
                    } else {
                        teamColor = TeamColor.BLUE;
                    }
                    optionsFile.set(Opts.TEAM_COLOR.s, teamColor);
                    saveOptionsFile();
                }
            }

            if(driver1.right_bumper.justPressed()) {
                if(values[index] == Opts.INITTIAL_AUTO_DELAY) {
                    double initialAutoDelay = optionsFile.get(Opts.INITTIAL_AUTO_DELAY.s, initialAutoDelayDefault);
                    initialAutoDelay +=1;
                    Utility.limit(initialAutoDelay, 0, 15);
                    optionsFile.set(Opts.INITTIAL_AUTO_DELAY.s, initialAutoDelay);
                    saveOptionsFile();
                }
            }
            if(driver1.left_bumper.justPressed()) {
                if(values[index] == Opts.INITTIAL_AUTO_DELAY) {
                    double initialAutoDelay = optionsFile.get(Opts.INITTIAL_AUTO_DELAY.s, initialAutoDelayDefault);
                    initialAutoDelay -=1;
                    Utility.limit(initialAutoDelay, 0, 15);
                    optionsFile.set(Opts.INITTIAL_AUTO_DELAY.s, initialAutoDelay);
                    saveOptionsFile();
                }
            }
            if(driver1.right_bumper.justPressed()) {
                if(values[index] == Opts.START_POSITION) {
                    StartingPosition startingPosition = optionsFile.get(Opts.START_POSITION.s, startingPositionDefault);
                    if (startingPosition == StartingPosition.LEFT) {
                        startingPosition = StartingPosition.RIGHT;
                    } else {
                        startingPosition = StartingPosition.LEFT;
                    }
                    optionsFile.set(Opts.START_POSITION.s, startingPosition);
                    saveOptionsFile();
                }
            }
//        if(driver1.right_bumper.justPressed()) {
//            if (values[index] == Opts.WAIT_TIME) {
//                double waitTime = optionsFile.get(wait, waitDefault);
//                waitTime += 0.25;
//                waitTime = Utility.limit(waitTime, 0.00, 15.0);
//                optionsFile.set(Opts.WAIT_TIME.s, waitTime);
//            } else {
//                optionsFile.set(values[index].s, true);
//            }
//        }
//            if(values[index] == Opts.WAIT_TIME) {
//                telemetry.addData("currentValue", optionsFile.get(values[index].s,waitDefault));
//            }
//            else {
//                telemetry.addData("currentValue", optionsFile.get(values[index].s,false));
//            }

            telemetry.addData(teamColorTag, optionsFile.get(Opts.TEAM_COLOR.s, teamColorDefault));



            telemetry.addData(initialAutoDelayTag, optionsFile.get(Opts.INITTIAL_AUTO_DELAY.s, initialAutoDelayDefault));



        }

        public double pow10floor(double x) {
            return Math.pow(10, Math.floor(Math.log(x) / Math.log(10)));
        }

        @Override
        protected Function getJoystickScalingFunction() {
            return Functions.none();
        }

        public enum Opts {
            TEAM_COLOR(teamColorTag), INITTIAL_AUTO_DELAY(initialAutoDelayTag), START_POSITION(startingPositionTag);

            //        public boolean b;
//        public double f;
            public String s;

            Opts(String s) {
                this.s = s;
            }
        }

    }



