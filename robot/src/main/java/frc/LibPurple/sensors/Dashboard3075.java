/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.sensors;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.LibPurple.swerve.Utils.Utils;

/**
 * Add your docs here.
 */
public class Dashboard3075 {
    static List<Param3075> params = new ArrayList<>();

    static FileWriter handle;
    static String text = "";

    final static String path = "/Dashboard/logs.csv";

    private static boolean isInParams(String param){
        for(int i = 0; i < params.size(); i++){
            if (params.get(i).nameY.equals(param)){
                return true;
            }
        }
        return false;
    }

    private static int getParam(String name){
        for(int i = 0; i < params.size(); i++){
            if (params.get(i).nameY.equals(name)){
                return i;
            }
        }
        return -1;
    }

    private static int getSize(){
        int maxSize = 0;
        for(int i = 0; i < params.size(); i++){
            if (maxSize < params.get(i).values.size()){
                maxSize = params.get(i).values.size();
            }
        }
        return maxSize;
    }

    public static void putNumber(String nameX, double valueX, String nameY, double valueY){
        if (!isInParams(nameY)){
            params.add(new Param3075(nameX, nameY));
        }
        try{
            params.get(getParam(nameY)).addValue(valueX, valueY);
        } catch (Exception e){
            Utils.print(e.toString());
        }
    }

    public static void putNumber(String name, double value){
        putNumber(name + "Time", Timer.getFPGATimestamp(), name, value);
    }

    private static void addStartLine(){
        for(int i = 0; i < params.size(); i++){
            text += params.get(i).nameX + " \t," + params.get(i).nameY + "\t,";
        }
        text += "\n";
    }

    private static void addValuesLines(){
        DriverStation.reportError("size" + getSize(), true);
        for(int index = 0; index < getSize(); index++){
            for(int i = 0; i < params.size(); i++){
                if (index < params.get(i).values.size()){
                    text += params.get(i).values.get(index).x + "\t," + params.get(i).values.get(index).y + "\t,";
                }else{
                    text += "\t,\t,";
                }
            }
            text += "\n";
        }
    }

    private static void writeText(){
        try{
            handle.write(text);
        } catch (IOException e){
            Utils.print(e.toString());
        }
    }

    public static void writeFile(){
        try{
            handle = new FileWriter(path, false);
            addStartLine();
            addValuesLines();
            writeText();
            handle.close();
        } catch (IOException e){
            Utils.print(e.toString());
        }
    }

    private static void removeAll(){
        for(int i = params.size() - 1; i >= 0; i--){
            params.remove(i);
        }
    }

    public static void deleteFile(){
        try{
            File f = new File(path);
            f.delete();
        } catch (Exception e){
            Utils.print(e.toString());
        }
        text = "";
        removeAll();
    }
}
