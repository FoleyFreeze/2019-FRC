package frc.robot.io;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Component;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class Inputs extends Component {
   
    public Joystick controlBoard;
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
    public boolean releaseDisk; 

    public boolean climb;
    public boolean reverseClimb;

    public boolean manualElevatorUp;
    public boolean manualElevatorDown; 
    public boolean autoElevator;
    public Elevator.ElevatorPosition elevatorTarget = ElevatorPosition.DONT_MOVE;
    //elevatorTarget means you press a button and it moves to a specific place
    public boolean ballNotHatch;
    
    public boolean shift;
    public boolean pitMode;

    public boolean leftNotRight;

    public boolean fourBarIn;
    public boolean fourBarOut;

    public boolean visionCargo;
    public boolean visionTarget;

    public byte[] xDixon;//history of joystick directions
    public byte[] yDixon;//history of joystick directions
    public byte[] rotDixon;//history of joystick directions

    public enum RocketCargoshipPosition {
        HI, MID, LO, FRONT, DEFAULT
    }
    public RocketCargoshipPosition rocketCargoState;
    public enum NearFarCargo {
        NEAR, FAR, CARGO, DEFAULT
    }
    public NearFarCargo nearFarCargo;

    public Inputs() {
        controlBoard = new Joystick(ElectroJendz.CONTROL_BOARD);
        gamePad = new Joystick(ElectroJendz.GAMEPAD);
        xDixon = new byte[k.IN_DixonSize];
        yDixon = new byte[k.IN_DixonSize];
        rotDixon = new byte[k.IN_DixonSize];
        nearFarCargo = NearFarCargo.DEFAULT;
        rocketCargoState = RocketCargoshipPosition.DEFAULT;
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

        ballNotHatch = controlBoard.getRawButton(cb.ballOrHatch);
        leftNotRight = controlBoard.getRawButton(cb.lOrR);

        
        //set buttons
        if(!k.DRV_disable){
            compassDrive = gamePad.getRawButton(bm.compassDrive);
            fieldOriented = gamePad.getRawButton(bm.fieldOriented);
            dodgingL = gamePad.getRawAxis(k.IN_dodgingL) > k.IN_DodgingMin;
            dodgingR = gamePad.getRawAxis(k.IN_dodgingR) > k.IN_DodgingMin;
            visionCargo = gamePad.getRawButton(4);
        }

        //flipOrientation = gamePad.getRawButton(k.IN_flipOrientation);
        pitMode = !controlBoard.getRawButton(cb.pitMode);
        shift = controlBoard.getRawButton(cb.shift);
        SmartDashboard.putBoolean("Pit Mode", pitMode);

        parseControlBoard();

        if(!k.ELE_disable){
            autoElevator = true;
            manualElevatorUp = gamePad.getRawButton(2);
            manualElevatorDown = gamePad.getRawButton(3);
            if(autoElevator){
                if(gamePad.getRawButton(3))elevatorTarget = ElevatorPosition.ROCKET_1_HATCH;
                else if(gamePad.getRawButton(2))elevatorTarget = ElevatorPosition.ROCKET_3_HATCH;
                else elevatorTarget = ElevatorPosition.DONT_MOVE;
            }
        }

        if(!k.GTH_disableDisk && !ballNotHatch) {
            diskGather = controlBoard.getRawButton(cb.gather);
            releaseDisk = controlBoard.getRawButton(cb.shoot);
        }

        if(!k.GTH_disableBall && ballNotHatch){
            ballGather = controlBoard.getRawButton(cb.gather);
            releaseBall = controlBoard.getRawButton(cb.shoot);
        }

        if(!k.CLM_disable){
            climb = controlBoard.getRawButton(cb.climb);
            reverseClimb = (climb && shift);
        }

        if(sense.isDisabled) {
            //when disabled require a new button press before moving
            elevatorTarget = ElevatorPosition.DONT_MOVE;
        }
        SmartDashboard.putString("Elevator Location", elevatorTarget.name());

        if(flipOrientation){
            flipOrientation();
        }
        SmartDashboard.putBoolean("Flip Orientation", flipOrientation);

        if(compassDrive){
            compassDrive();
        }
        SmartDashboard.putBoolean("Compass Drive", compassDrive);

        
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

    private void parseControlBoard(){
        if(controlBoard.getRawButton(cb.high)){
            rocketCargoState = RocketCargoshipPosition.HI;
        }else if(controlBoard.getRawButton(cb.middle)){
            rocketCargoState = RocketCargoshipPosition.MID;
        }else if(controlBoard.getRawButton(cb.low)){
            rocketCargoState = RocketCargoshipPosition.LO;
        }else if(controlBoard.getRawButton(cb.front)){
            rocketCargoState = RocketCargoshipPosition.FRONT;
        }

        switch(rocketCargoState){
            case HI:
            controlBoard.setOutput(cb.high, true);
            controlBoard.setOutput(cb.middle, false);
            controlBoard.setOutput(cb.low, false);
            controlBoard.setOutput(cb.front, false);
            break;

            case MID:
            controlBoard.setOutput(cb.high, false);
            controlBoard.setOutput(cb.middle, true);
            controlBoard.setOutput(cb.low, false);
            controlBoard.setOutput(cb.front, false);
            break;

            case LO:
            controlBoard.setOutput(cb.high, false);
            controlBoard.setOutput(cb.middle, false);
            controlBoard.setOutput(cb.low, true);
            controlBoard.setOutput(cb.front, false);
            break;

            case FRONT:
            controlBoard.setOutput(cb.high, false);
            controlBoard.setOutput(cb.middle, false);
            controlBoard.setOutput(cb.low, false);
            controlBoard.setOutput(cb.front, true);
            break;   
            
            case DEFAULT:
            controlBoard.setOutput(cb.high, true);
            controlBoard.setOutput(cb.middle, true);
            controlBoard.setOutput(cb.low, true);
            controlBoard.setOutput(cb.front, true);
            break;
        }
        

        if(controlBoard.getRawButton(cb.farRkt)){
            nearFarCargo = NearFarCargo.FAR;
        }else if(controlBoard.getRawButton(cb.nearRkt)){
            nearFarCargo = NearFarCargo.NEAR;
        }else if(controlBoard.getRawButton(cb.cargoShip)){
            nearFarCargo = NearFarCargo.CARGO;
        }

        switch(nearFarCargo){
            case FAR:
            controlBoard.setOutput(cb.farRkt, true);
            controlBoard.setOutput(cb.nearRkt, false);
            controlBoard.setOutput(cb.cargoShip, false);
            break;

            case NEAR:
            controlBoard.setOutput(cb.farRkt, false);
            controlBoard.setOutput(cb.nearRkt, true);
            controlBoard.setOutput(cb.cargoShip, false);
            break;

            case CARGO:
            controlBoard.setOutput(cb.farRkt, false);
            controlBoard.setOutput(cb.nearRkt, false);
            controlBoard.setOutput(cb.cargoShip, true);
            break;  

            case DEFAULT:
            controlBoard.setOutput(cb.farRkt, true);
            controlBoard.setOutput(cb.nearRkt, true);
            controlBoard.setOutput(cb.cargoShip, true);
            break;
        }
    }

}