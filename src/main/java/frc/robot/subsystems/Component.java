package frc.robot.subsystems;

import frc.robot.io.Inputs;
import frc.robot.io.Outputs;
import frc.robot.io.Vision;

public class Component {
    public static Inputs in;
    public static Outputs out; 
    public static Sensors sense; 
    public static Vision view; 
    public static BallGatherer grabCargo;
    public static Climber climb; 
    public static Elevator elevator;
    public static DiskGatherer grabDisk;
    public static DriveTrain drive;

    public static void initAll() {
        in = new Inputs();
        out = new Outputs(); 
        sense = new Sensors();
        view = new Vision();
        grabCargo = new BallGatherer();
        climb = new Climber();
        elevator = new Elevator();
        grabDisk = new DiskGatherer();
        drive = new DriveTrain(); 
    }

    public static void runAll(){
        in.run();
        out.run();
        sense.run();
        view.run();
        grabCargo.run();
        climb.run();
        elevator.run();
        grabDisk.run();
        drive.run();
    }

}