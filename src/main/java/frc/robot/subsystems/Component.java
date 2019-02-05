package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.io.Inputs;
import frc.robot.io.K_Competition_Bot;
import frc.robot.io.K_Swerve_Bot;
import frc.robot.io.Calibrations;
import frc.robot.io.Outputs;
import frc.robot.io.OutputsCompBot;
import frc.robot.io.OutputsSwerveBot;
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
    public static Calibrations k;

    public static void initAll() {
        switch(Calibrations.BOT_Version){
            case COMPETITION:
            case PRACTICE:
                k = new K_Competition_Bot();
                break;
            case SWERVE_BOT:
                k = new K_Swerve_Bot();
                break;
        }
        
        in = new Inputs();
        sense = new Sensors();
        view = new Vision();
        grabCargo = new BallGatherer();
        climb = new Climber();
        elevator = new Elevator(); //F
        grabDisk = new DiskGatherer();
        drive = new DriveTrain(); 

        switch(Calibrations.BOT_Version){
            case COMPETITION:
            case PRACTICE:
                out = new OutputsCompBot();
                break;
            case SWERVE_BOT:
                out = new OutputsSwerveBot();
                break;
        }
    }

    public static void runAll(){
        in.run();
        sense.run();
        view.run();
        //grabCargo.run();
        //climb.run();
        //elevator.run();
        //grabDisk.run();
        drive.run();
        out.run();
    }

}