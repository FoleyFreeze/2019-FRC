package frc.robot.io;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Component;

public class Inputs extends Component {
   
    private final Joystick controlBoard;
    private final Joystick gamePad;
        
    //----------------------------------Driver Functions--------------------------------
    public double xAxisDrive; 
    public double yAxisDrive;
    public double rotAxisDrive;
    public boolean resetEncoders;
    public boolean fieldOriented;
    public boolean compassDrive;
    public boolean dodgingL;
    public boolean dodgingR;
    //----------------------------Operator Functions-------------------------------------- 
    public boolean ballGather;
    public boolean releaseBall; 
    public boolean diskGather;
    public boolean releaseDisc; 

    public boolean climb;
    public boolean reverseclimb;

    public boolean manualElevatorUp;
    public boolean manualElevatorDown; 
    public boolean autoElevator;
    public Elevator.ElevatorPosition elevatorTarget;
    //elevatorTarget means you press a button and it moves to a specific place
    
    public boolean pitMode;

    public Inputs() {
        controlBoard = new Joystick(ElectroJendz.CONTROL_BOARD);
        gamePad = new Joystick(ElectroJendz.GAMEPAD);
    }

    

    public void run() {
        if(gamePad.getRawButton(k.IN_resetGyro)){
            sense.init();
        }

     
        //read joystick
        xAxisDrive = -gamePad.getRawAxis(k.IN_xDriveAxis);
        yAxisDrive = gamePad.getRawAxis(k.IN_yDriveAxis);
        rotAxisDrive = -gamePad.getRawAxis(k.IN_rotDriveAxis);

        //deadband if tiny
        if(Math.abs(xAxisDrive) < k.IN_xDeadband) xAxisDrive = 0; 
        if(Math.abs(yAxisDrive) < k.IN_xDeadband) yAxisDrive = 0; 
        if(Math.abs(xAxisDrive) < k.IN_xyDeadband && Math.abs(yAxisDrive) < k.IN_xyDeadband) {
            xAxisDrive = 0;
            yAxisDrive = 0;
        }
        if(Math.abs(rotAxisDrive) < k.IN_rotDeadband) rotAxisDrive = 0;//if command is unreasonably tiny, don't turn

        //set buttons
        compassDrive = gamePad.getRawButton(k.IN_compassDrive);
        fieldOriented = gamePad.getRawButton(k.IN_fieldOriented);
        pitMode = gamePad.getRawButton(k.IN_pitMode);
        diskGather = gamePad.getRawButton(k.IN_diskGather);
        dodgingL = gamePad.getRawAxis(k.IN_dodgingL) > k.IN_DodgingMin;
        dodgingR = gamePad.getRawAxis(k.IN_dodgingR) > k.IN_DodgingMin;

        if(compassDrive){
            compassDrive();
        }
    }

    public void compassDrive(){
        //x and y to theta and r
        double theta = Math.atan2(yAxisDrive, xAxisDrive);
        double r = Math.sqrt(xAxisDrive * xAxisDrive + yAxisDrive * yAxisDrive);
        theta = theta/45;
        Math.round(theta);
        theta = theta * 45;
        yAxisDrive = r * Math.sin(theta);
        xAxisDrive = r * Math.cos(theta);
    }

}