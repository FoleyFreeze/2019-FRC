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
        if(gamePad.getRawButton(K.IN_resetGyro)){
            sense.init();//reset navx angle
        }

     
        //read joystick
        xAxisDrive = -gamePad.getRawAxis(K.IN_xDriveAxis);
        yAxisDrive = gamePad.getRawAxis(K.IN_yDriveAxis);
        rotAxisDrive = -gamePad.getRawAxis(K.IN_rotDriveAxis);

        //deadband if tiny
        if(Math.abs(xAxisDrive) < K.IN_xDeadband) xAxisDrive = 0; 
        if(Math.abs(yAxisDrive) < K.IN_xDeadband) yAxisDrive = 0; 
        if(Math.abs(xAxisDrive) < K.IN_xyDeadband && Math.abs(yAxisDrive) < K.IN_xyDeadband) {
            xAxisDrive = 0;
            yAxisDrive = 0;
        }
        if(Math.abs(rotAxisDrive) < K.IN_rotDeadband) rotAxisDrive = 0;//if command is unreasonably tiny, don't turn

        //set buttons
        compassDrive = gamePad.getRawButton(K.IN_compassDrive);
        fieldOriented = gamePad.getRawButton(K.IN_fieldOriented);
        pitMode = gamePad.getRawButton(K.IN_pitMode);
        diskGather = gamePad.getRawButton(K.IN_diskGather);

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