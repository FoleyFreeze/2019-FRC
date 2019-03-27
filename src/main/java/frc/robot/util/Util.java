package frc.robot.util;

import frc.robot.subsystems.autodrive.Point;

public class Util{

    public static double absMax(double[] data) {
        double max = Math.abs(data[0]);
        for(double d : data) max = Math.max(max, Math.abs(d));
        return max;
    }

    //1d interplation.  requires axis to be constantly increasing
    public static double interpolate(double[] axis, double[] table, double value){
        
        if(value <= axis[0]) return table[0];
        else if(value >= axis[axis.length-1]) return table[table.length-1];

        int i=0;
        for(;i<axis.length;i++){
            if(value < axis[i]){
                break;
            }
        }
        double frac = (value - axis[i-1])/(axis[i]-axis[i-1]);
        return frac*(table[i] - table[i-1]) + table[i-1];
    }
    
    public static double limit(double value, double limit){
        return Math.max(-limit, Math.min(limit, value));
    }
    
    //calculate the distance between 2 points
    public static double dist(Point p1, Point p2){
        double diffX = p1.x - p2.x;
        double diffY = p1.y - p2.y;
        return Math.sqrt(diffX*diffX + diffY*diffY);
    }

    public static double deadband(double value, double deadband){
        //zero if in the deadband zone
        if(Math.abs(value) < deadband) return 0;

        //else normalize the remaining part of the axis to 0 - 1
        return Math.signum(value) * (Math.abs(value) - deadband) / (1 - deadband);
    }
}
