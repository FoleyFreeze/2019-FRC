package frc.robot.util;

public class Angle{
    private double angle;

    public double get(){
        return angle;
    }

    public void set(double newAngle){
        angle = limit(newAngle);
    }

    public void set(Angle newAngle){
        angle = limit(newAngle.get());
    }

    private double limit(double value){
        while(value > 180)value -= 360;
        while(value < -180)value += 360;
        return value;
    }

    public double sub(double subtrahend){
        return limit(angle - subtrahend);
    }

    public double sub(Angle subtrahend){
        return limit(angle - subtrahend.get());
    }

    public double add(double adder){
        return limit(angle + adder);
    }

    public double add(Angle adder){
        return limit(angle + adder.get());
    }
}