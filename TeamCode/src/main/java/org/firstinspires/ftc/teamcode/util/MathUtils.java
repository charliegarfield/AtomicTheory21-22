package org.firstinspires.ftc.teamcode.util;

public class MathUtils {
    public static double closestTo(double d, double... values){
        int lowestDif = 0;
        for(int i = 1; i < values.length; i++){
            if(Math.abs(values[lowestDif]-d) > Math.abs(values[i]-d)) lowestDif = i;
        }
        return values[lowestDif];
    }
    public static int closestTo(double d, int... values){
        int lowestDif = 0;
        for(int i = 1; i < values.length; i++){
            if(Math.abs(values[lowestDif]-d) > Math.abs(values[i]-d)) lowestDif = i;
        }
        return values[lowestDif];
    }
}
