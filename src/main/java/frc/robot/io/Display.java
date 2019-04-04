import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Display {

    public static void run(){
        Shuffleboard.getTab("CompTap").add("hasHatch",false);
        Shuffleboard.getTab("CompTap").add("hasCargo",false);
        Shuffleboard.getTab("CompTap").add("rseX",false);
        Shuffleboard.getTab("CompTap").add("rseY",false);
    }

}