package frc.LibPurple.sensors;

import java.util.ArrayList;
import java.util.List;

public class Param3075{
    String nameX;
    String nameY;
    List<Value> values = new ArrayList<>();

    public Param3075(String nameX, String nameY){
        this.nameX = nameX;
        this.nameY = nameY;
    }

    public void addValue(double x, double y){
        values.add(new Value(x, y));
    }

    public class Value{
        double x;
        double y;

        public Value(double x, double y){
            this.x = x;
            this.y = y;
        }
    }
}