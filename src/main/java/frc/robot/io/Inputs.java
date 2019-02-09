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
    public boolean flipOrientation;
    public boolean resetGyro;
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

    public byte[] xDixon;//history of joystick directions
    public byte[] yDixon;//history of joystick directions
    public byte[] rotDixon;//history of joystick directions

    public Inputs() {
        controlBoard = new Joystick(ElectroJendz.CONTROL_BOARD);
        gamePad = new Joystick(ElectroJendz.GAMEPAD);
        xDixon = new byte[k.IN_DixonSize];
        yDixon = new byte[k.IN_DixonSize];
        rotDixon = new byte[k.IN_DixonSize];
    }

    

    public void run() {
        resetGyro = gamePad.getRawButton(k.IN_resetGyro);
        if(resetGyro){
            sense.init();
        }
     
        //read joystick
        xAxisDrive = -gamePad.getRawAxis(k.IN_xDriveAxis);
        yAxisDrive = gamePad.getRawAxis(k.IN_yDriveAxis);
        rotAxisDrive = gamePad.getRawAxis(k.IN_rotDriveAxis);

        //deadband if tiny
        if(Math.abs(xAxisDrive) < k.IN_xDeadband) xAxisDrive = 0; 
        if(Math.abs(yAxisDrive) < k.IN_xDeadband) yAxisDrive = 0; 
        if(Math.abs(xAxisDrive) < k.IN_xyDeadband && Math.abs(yAxisDrive) < k.IN_xyDeadband) {
            xAxisDrive = 0;
            yAxisDrive = 0;
        }
        if(Math.abs(rotAxisDrive) < k.IN_rotDeadband) rotAxisDrive = 0;//if command is unreasonably tiny, don't turn

        if(dixonDetector(xDixon, xAxisDrive))xAxisDrive = 0;//stop Dixon from breaking robot
        if(dixonDetector(yDixon, yAxisDrive))yAxisDrive = 0;//stop Dixon from breaking robot
        if(dixonDetector(rotDixon, rotAxisDrive))rotAxisDrive = 0;//stop Dixon from breaking robot

        //set buttons
        compassDrive = gamePad.getRawButton(k.IN_compassDrive);
        fieldOriented = gamePad.getRawButton(k.IN_fieldOriented);
        //flipOrientation = gamePad.getRawButton(k.IN_flipOrientation);
        //pitMode = gamePad.getRawButton(k.IN_pitMode);
        //diskGather = gamePad.getRawButton(k.IN_diskGather);
        dodgingL = gamePad.getRawAxis(k.IN_dodgingL) > k.IN_DodgingMin;
        dodgingR = gamePad.getRawAxis(k.IN_dodgingR) > k.IN_DodgingMin;

        if(flipOrientation){
            flipOrientation();
        }

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

    public void flipOrientation(){
        xAxisDrive = -xAxisDrive;
        yAxisDrive = -yAxisDrive;
    }

    public boolean dixonDetector(byte[] prevDirs, double currVal){//check if Dixon is being stupid and bouncing the joysticks
        //store previous joystick direction as -1, 0, or 1
        for(int i = prevDirs.length - 1; i > 0; i--){
            prevDirs[i] = prevDirs[i-1];
        }
        if(currVal > 0) prevDirs[0] = 1;
        else if(currVal < 0) prevDirs[0] = -1;
        else prevDirs[0] = 0;
        //count number of direction changes in the last array.length
        byte curr = prevDirs[0];
        int count = 0;
        for(int i = 0; i< prevDirs.length; i++){
            if(prevDirs[i] != 0){
                if(curr == 0){
                   curr = prevDirs[i]; 
                } else if(prevDirs[i]!=curr){
                    count++;
                    curr = prevDirs[i];
                }
            }
        }
        //if more than 1 direction change, true
        return count>1;
    }

}