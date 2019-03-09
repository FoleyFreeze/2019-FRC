package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.ButtonMap;
import frc.robot.io.Calibrations;
import frc.robot.io.ControlBoard;
import frc.robot.io.Inputs;
import frc.robot.io.K_Competition_Bot;
import frc.robot.io.K_Swerve_Bot;
import frc.robot.io.Outputs;
import frc.robot.io.OutputsCompBot;
import frc.robot.io.OutputsSwerveBot;
import frc.robot.subsystems.vision.Vision;

public class Component {
    public static ButtonMap bm;
    public static Inputs in;
    public static Outputs out;
    public static Sensors sense;
    public static Vision view;
    public static CargoGatherer grabCargo;
    public static Climber climb; 
    public static Elevator elevator;
    public static HatchGatherer grabHatch;
    public static DriveTrain drive;
    public static Calibrations k;
    public static RSE rse;
    public static ControlBoard cb;
    public static LEDs leds;

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
        grabCargo = new CargoGatherer();
        climb = new Climber();
        elevator = new Elevator(); //F
        grabHatch = new HatchGatherer();
        drive = new DriveTrain(); 
        rse = new RSE();
        cb = new ControlBoard();
        leds = new LEDs();

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
        bm.set();
        in.run();
        sense.run();
        view.run();
        rse.run();
        grabCargo.run();
        climb.run();
        elevator.run();
        grabHatch.run();
        drive.run();
        out.run();
        leds.run();
        SmartDashboard.putString("Which bot?", Calibrations.BOT_Version.toString());
    }

}