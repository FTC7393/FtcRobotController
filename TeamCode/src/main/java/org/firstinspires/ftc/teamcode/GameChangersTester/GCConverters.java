package org.firstinspires.ftc.teamcode.GameChangersTester;

import ftc.electronvolts.util.files.BasicConverters;
import ftc.electronvolts.util.files.Converter;
import ftc.electronvolts.util.files.Converters;
import ftc.electronvolts.util.files.UtilConverters;
import ftc.evlib.util.EVConverters;

public class GCConverters extends UtilConverters {

    static {
        BasicConverters.converterMap.put(StartingPosition.class, new Converter<StartingPosition>() {

            @Override
            public String toString(StartingPosition object) {
                return object.name();
            }

            @Override
            public StartingPosition fromString(String string) {
                return StartingPosition.valueOf(string);
            }
        });
    }

    private static final Converters INSTANCE = new GCConverters();

    public static Converters getInstance() {
        return INSTANCE;
    }

    protected GCConverters() {
        super();
    }

}
