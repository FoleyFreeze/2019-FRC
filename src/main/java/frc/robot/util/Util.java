package frc.robot.util;

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
}
