package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.io.ButtonMap;
import frc.robot.io.Calibrations;
import frc.robot.io.Inputs;
import frc.robot.io.K_Competition_Bot;
import frc.robot.io.K_Swerve_Bot;
import frc.robot.io.Outputs;
import frc.robot.io.OutputsCompBot;
import frc.robot.io.OutputsSwerveBot;
import frc.robot.io.Vision;

public class Component {
    public static ButtonMap bm;
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
        
        // select RobotType based on jumper position
        DigitalInput id1 = new DigitalInput(8);
        DigitalInput id2 = new DigitalInput(9);

        if(!id1.get() && !id2.get()){
            Calibrations.BOT_Version = Calibrations.RobotType.SWERVE_BOT;
        } else if(!id1.get()) {
            Calibrations.BOT_Version = Calibrations.RobotType.COMPETITION;
        } else if(!id2.get()) {
            Calibrations.BOT_Version = Calibrations.RobotType.PRACTICE;
        } else {
            System.out.println("Error: The Robot Selection Jumper is Missing!");
            System.exit(-1);
        }
        id1.close();
        id2.close(); 

        //select button map
        bm = new ButtonMap();

        // picks the right subsystems based on RobotType 
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
        grabCargo.run();//UNCOMMENT WHEN ATTACHED
        climb.run();
        elevator.run();
        grabDisk.run();
        drive.run();
        out.run();
    }

}