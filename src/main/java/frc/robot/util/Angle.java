package frc.robot.util;


public class Angle{
    private double angle;

    public double getDeg(){
        return angle;
    }

    public double getRad(){
        return toRad(angle);
    }

    public void setDeg(double newAngle){
        angle = limit(newAngle);
    }
    
    public void setRad(double newAngle){
        angle = limit(toDeg(newAngle));
    }

    public void setDeg(Angle newAngle){
        angle = limit(newAngle.getDeg());
    }

    private double limit(double value){
        while(value > 180) value -= 360;
        while(value < -180) value += 360;
        return value;
    }

    public double subDeg(double subtrahend){
        return limit(angle - subtrahend);
    }
    
    public double subRad(double subtrahend){
        return limit(angle - toDeg(subtrahend));
    }

    public double subDeg(Angle subtrahend){
        return limit(angle - subtrahend.getDeg());
    }

    public double subtrahendDeg(double minuend){
        return limit(minuend-angle);
    }

    public double subtrahendRad(double minuend){
        return limit(toDeg(minuend) - angle);
    }

    public double subtrahendDeg(Angle minuend){
        return limit(minuend.getDeg()-angle);
    }

    public double addDeg(double adder){
        return limit(angle + adder);
    }

    public double addRad(double adder){
        return limit(angle + toDeg(adder));
    }

    public double addDeg(Angle adder){
        return limit(angle + adder.getDeg());
    }

    public static double toRad(double deg) {
        return deg * Math.PI/180;
    }
    
    public static double toDeg(double rad) {
        return rad * 180/Math.PI;
    }
}