package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.Calibrations;
import frc.robot.io.Calibrations.RobotType;
import frc.robot.io.Display;
import frc.robot.io.Inputs;
import frc.robot.io.K_Competition_Bot;
import frc.robot.io.K_Swerve_Bot;
import frc.robot.io.Outputs;
import frc.robot.io.OutputsCompBot;
import frc.robot.io.OutputsSwerveBot;
import frc.robot.subsystems.autodrive.AutoDrive;
import frc.robot.subsystems.autodrive.Pathfinder;
import frc.robot.subsystems.vision.Vision;

public class Component {
    public static Inputs in;
    public static Outputs out;
    public static Sensors sense;
    public static Vision view;
    public static Climber climber; 
    public static Elevator elevator;
    public static ArmGatherer gatherer;
    public static DriveTrain drive;
    public static Calibrations k;
    public static RSE rse;
    public static LEDs leds;
    public static Pathfinder pathfinder;
    public static AutoDrive autoDriving;
	public static Display display;

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
            System.out.println("Defaulting to COMP_BOT!");
            Calibrations.BOT_Version = RobotType.COMPETITION;
            //System.exit(-1);
        }
        id1.close();
        id2.close(); 

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
        pathfinder = new Pathfinder();
        autoDriving = new AutoDrive();
        climber = new Climber();
        elevator = new Elevator();
        if(k.SCR_ScorpioSelected){
            gatherer = new ScorpioGatherer();
        } else {
            gatherer = new ArmGatherer();
        }
        drive = new DriveTrain(); 
        rse = new RSE();
        leds = new LEDs();
		display = new Display();

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
        rse.run();
        autoDriving.run();
        climber.run();
        elevator.run();
        gatherer.run();
        drive.run();
        out.run();
        leds.run();
	    display.run();
        //SALwasHere-SmartDashboard.putString("Which bot?", Calibrations.BOT_Version.toString());
    }

}