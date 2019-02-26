package frc.robot.io;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Component;

public class Inputs extends Component {
   
    private final Joystick controlBoard;
    public Joystick gamePad;
        
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
    public boolean reverseClimb;

    public boolean manualElevatorUp;
    public boolean manualElevatorDown; 
    public boolean autoElevator;
    public Elevator.ElevatorPosition elevatorTarget = ElevatorPosition.DONT_MOVE;
    //elevatorTarget means you press a button and it moves to a specific place
    public boolean rocketL1;
    public boolean rocketL2;
    public boolean rocketL3;
    public boolean rocketSideLeft;
    public boolean rocketSideRight;
    
    public boolean pitMode;

    public boolean fourBarIn;
    public boolean fourBarOut;

    public boolean visionCargo;
    public boolean visionTarget;

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
        resetGyro = gamePad.getRawButton(bm.resetGyro);
        if(resetGyro){
            sense.init();
        }
     
        //read joystick
        xAxisDrive = -gamePad.getRawAxis(bm.IN_xDriveAxis);
        yAxisDrive = gamePad.getRawAxis(bm.IN_yDriveAxis);
        rotAxisDrive = gamePad.getRawAxis(bm.IN_rotDriveAxis);

        //deadband if tiny
        if(Math.abs(xAxisDrive) < bm.xDeadband) xAxisDrive = 0; 
        if(Math.abs(yAxisDrive) < bm.xDeadband) yAxisDrive = 0; 
        if(Math.abs(xAxisDrive) < bm.xyDeadband && Math.abs(yAxisDrive) < bm.xyDeadband) {
            xAxisDrive = 0;
            yAxisDrive = 0;
        }
        if(Math.abs(rotAxisDrive) < bm.rotDeadband) rotAxisDrive = 0;//if command is unreasonably tiny, don't turn

        String stopDixon;//for joy of programming
        boolean dixon;//for joy of programming

        if(dixonDetector(xDixon, xAxisDrive))xAxisDrive = 0;//stop Dixon from breaking robot
        if(dixonDetector(yDixon, yAxisDrive))yAxisDrive = 0;//stop Dixon from breaking robot
        if(dixonDetector(rotDixon, rotAxisDrive))rotAxisDrive = 0;//stop Dixon from breaking robot
        
        if(dixonDetector(xDixon, xAxisDrive)||dixonDetector(yDixon, yAxisDrive)||dixonDetector(rotDixon, rotAxisDrive)){
            dixon = true;
        }else{
            dixon = false;
        }

        if(dixon)stopDixon = "Stop messing with the joysticks, Dixon!";
        else stopDixon = "   ";

        SmartDashboard.putString("Dixon Detector", stopDixon);

        //set buttons
        if(!k.DRV_disable){
            compassDrive = gamePad.getRawButton(bm.compassDrive);
            fieldOriented = gamePad.getRawButton(bm.fieldOriented);
            dodgingL = gamePad.getRawAxis(k.IN_dodgingL) > k.IN_DodgingMin;
            dodgingR = gamePad.getRawAxis(k.IN_dodgingR) > k.IN_DodgingMin;
            visionCargo = gamePad.getRawButton(4);
        }

        //flipOrientation = gamePad.getRawButton(k.IN_flipOrientation);
        //pitMode = gamePad.getRawButton(k.IN_pitMode);
        //diskGather = gamePad.getRawButton(k.IN_diskGather);
        if(!k.ELE_disable){
            rocketL1 = gamePad.getRawButton(k.IN_rocketL1);
            rocketL2 = gamePad.getRawButton(k.IN_rocketL2);
            rocketL3 = gamePad.getRawButton(k.IN_rocketL3);
            rocketSideLeft = gamePad.getRawButton(k.IN_rocketSideLeft);
            rocketSideRight = gamePad.getRawButton(k.IN_rocketSideRight);
            autoElevator = true;
            manualElevatorUp = gamePad.getRawButton(2);
            manualElevatorDown = gamePad.getRawButton(3);
            if(autoElevator){
                if(gamePad.getRawButton(2))elevatorTarget = ElevatorPosition.ROCKET_1_HATCH;
                else if(gamePad.getRawButton(3))elevatorTarget = ElevatorPosition.ROCKET_3_HATCH;
                else elevatorTarget = ElevatorPosition.DONT_MOVE;
            }
        }

        if(!k.CLM_disable){
            climb = gamePad.getRawButton(2);
            reverseClimb = gamePad.getRawButton(3);
        }

        String elevatorState;

        if(sense.isDisabled) {
            //when disabled require a new button press before moving
            elevatorTarget = ElevatorPosition.DONT_MOVE;
            elevatorState = "irrelevant";
        } else if(rocketL3 && sense.hasBall){
            elevatorTarget = ElevatorPosition.ROCKET_3_CARGO;
            elevatorState = "Level 3 Cargo";
        } else if (rocketL3 && sense.hasHatch) {
            elevatorTarget = ElevatorPosition.ROCKET_3_HATCH;
            elevatorState = "Level 3 Hatch";
        } else if (rocketL2 && sense.hasBall) {
            elevatorTarget = ElevatorPosition.ROCKET_2_CARGO;
            elevatorState = "Level 2 Cargo";
        } else if (rocketL2 && sense.hasHatch) {
            elevatorTarget = ElevatorPosition.ROCKET_2_HATCH;
            elevatorState = "Level 2 Hatch";
        } else if (rocketL1 && sense.hasBall) {
            elevatorTarget = ElevatorPosition.ROCKET_1_CARGO;
            elevatorState = "Level 1 Cargo";
        } else if (rocketL1 && sense.hasHatch) {
            elevatorTarget = ElevatorPosition.ROCKET_1_HATCH;
            elevatorState = "Level 2 Hatch";
        }else{
            elevatorState = "unknown";
        }
        SmartDashboard.putString("Elevator Location", elevatorState);

        if(flipOrientation){
            flipOrientation();
        }
        SmartDashboard.putBoolean("Flip Orientation", flipOrientation);

        if(compassDrive){
            compassDrive();
        }
        SmartDashboard.putBoolean("Compass Drive", compassDrive);

        SmartDashboard.putBoolean("Pit Mode", pitMode);
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
   
    public enum States { BALLSCORE, HATCHSCORE, HATCHHUMAN, HATCHGROUND, BALLGROUND, IDLE };
    
        private void autoGatherStates() {
            
        }

}