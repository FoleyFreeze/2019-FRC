package frc.robot.util;

public class Util{

    public static double absMax(double[] data) {
        double max = Math.abs(data[0]);
        for(double d : data) max = Math.max(max, Math.abs(d));
        return max;
    }
}
