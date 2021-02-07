package org.firstinspires.ftc.teamcode.GameChangersTester;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import ftc.evlib.util.FileUtil;

public class VuforiaKeyReader {


    public static String readVuforiaKey(){
        try {
            String VUFORIA_KEY;
            File keyFile = FileUtil.getAppFile("vuforiakey.txt");
            BufferedReader breader = new BufferedReader(new FileReader(keyFile));
            String line = breader.readLine();
            breader.close();
            VUFORIA_KEY = line + " \n";
            return VUFORIA_KEY;
        } catch (IOException i) {
            throw new RuntimeException("force stop because vuforia key was not found", i);
        }
    }

}
