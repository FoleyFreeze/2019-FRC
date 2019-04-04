package frc.robot.io;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Display {

    public static void run(){
        Shuffleboard.getTab("CompTab").add("hasHatch",false);
        Shuffleboard.getTab("CompTab").add("hasCargo",false);
        Shuffleboard.getTab("CompTab").add("rseX",0.0);
        Shuffleboard.getTab("CompTab").add("rseY",0.0);
    }

}