package frc.robot.io;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Component;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.vision.VisionData;

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
    public boolean resetEleEnc;
    //----------------------------Operator Functions-------------------------------------- 
    public boolean cargoGather;
    public boolean releaseCargo; 
    public boolean hatchGather;
    public boolean releaseHatch; 

    public boolean climb;
    public boolean reverseClimb;

    public boolean manualElevatorUp;
    public boolean manualElevatorDown; 
    public boolean autoElevator;
    public boolean elevatorStage;
    public Elevator.ElevatorPosition elevatorTarget = ElevatorPosition.DONT_MOVE;
    public boolean cargoNotHatch;
    
    public boolean shift;
    public boolean pitMode;
    public boolean shoot;
    public boolean gather;

    public boolean leftNotRight;
    public boolean autoNotManualMode;

    public boolean fourBarIn;
    public boolean fourBarOut;

    public boolean visionCargo;
    public boolean visionTarget;

    public boolean actionCargo;
    public boolean actionHatch;
    public boolean camLightsOn;
    public boolean enableCamera;

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
        elevatorTarget = ElevatorPosition.DONT_MOVE;
    }

    

    public void run() {
        resetGyro = gamePad.getRawButton(bm.resetGyro);
        if(resetGyro){
            sense.init();
        }

        resetEleEnc = gamePad.getRawButton(11) && gamePad.getRawButton(15);
        if(resetEleEnc){
            out.resetEleEnc();
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
        
        if(dixonDetector(xDixon, xAxisDrive)
                ||dixonDetector(yDixon, yAxisDrive)
                ||dixonDetector(rotDixon, rotAxisDrive)){
            dixon = true;
        }else{
            dixon = false;
        }

        if(dixon)stopDixon = "Stop messing with the joysticks, Dixon!";
        else stopDixon = "   ";

        SmartDashboard.putString("Dixon Detector", stopDixon);

        cargoNotHatch = controlBoard.getRawButton(cb.cargoOrHatch);
        leftNotRight = controlBoard.getRawButton(cb.lOrR);
        autoNotManualMode = leftNotRight;

        
        //set buttons
        if(!k.DRV_Disable){
            compassDrive = gamePad.getRawButton(bm.compassDrive);
            fieldOriented = gamePad.getRawButton(bm.fieldOriented);
            //dodgingL = gamePad.getRawAxis(k.IN_dodgingL) > k.IN_DodgingMin;
            //dodgingR = gamePad.getRawAxis(k.IN_dodgingR) > k.IN_DodgingMin;
            //visionCargo = gamePad.getRawButton(4);
        }
        actionCargo = gamePad.getRawAxis(k.IN_dodgingL) > k.IN_DodgingMin;
        actionHatch = gamePad.getRawAxis(k.IN_dodgingR) > k.IN_DodgingMin;
        camLightsOn = false;
        enableCamera = false;

        //flipOrientation = gamePad.getRawButton(k.IN_flipOrientation);
        pitMode = !controlBoard.getRawButton(cb.pitMode);
        shift = controlBoard.getRawButton(cb.shift);
        shoot = controlBoard.getRawButton(cb.shoot);
        gather = controlBoard.getRawButton(cb.gather);
        SmartDashboard.putBoolean("Pit Mode", pitMode);

        parseControlBoard();

        SmartDashboard.putBoolean("ActionCargo", actionCargo);
        SmartDashboard.putBoolean("ActionHatch", actionHatch);

        autoElevator = true;
        releaseCargo = false;
        releaseHatch = false;
        hatchGather = false;
        cargoGather = false;
        elevatorStage = false;
        visionTarget = false;
        visionCargo = false;

        /*
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

        //remove once robot state is working
        if(!k.GTH_disableHatch && !cargoNotHatch) {
            hatchGather = controlBoard.getRawButton(cb.gather);
            releaseHatch = controlBoard.getRawButton(cb.shoot);
        }
        //remove once robot state is working
        if(!k.GTH_disableCargo && cargoNotHatch){
            cargoGather = controlBoard.getRawButton(cb.gather);
            releaseCargo = controlBoard.getRawButton(cb.shoot);
        }
        */

        if(!k.CLM_disable){
            boolean temp = controlBoard.getRawButton(cb.climb);
            climb = temp && !shift;
            reverseClimb = (temp && shift);
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

        SmartDashboard.putBoolean("Pit Mode", pitMode);

        if(autoNotManualMode){
            // autofunctions disable cals and cargo or hatch detection
            if(actionCargo) {
                autoGather(!sense.hasCargo);
                autoScore(sense.hasCargo);
            } else if(actionHatch) {
                autoGather(!sense.hasHatch);
                autoScore(sense.hasHatch);
            } else {
                autoGather(false);
                autoScore(false);
            }

            SmartDashboard.putString("AutoGatherState",autoGatherState.name());
            SmartDashboard.putString("AutoScoreState",autoScoreState.name());

            SmartDashboard.putBoolean("GoodCargoVision",view.goodCargoImage());
            SmartDashboard.putBoolean("GoodVisionTarget",view.goodVisionTarget());
        } else {
            //manual mode
            if (cargoNotHatch){
                cargoGather = gather;
                releaseCargo = shoot;
                hatchGather = false;
                releaseHatch = false;

                visionCargo = actionCargo && !sense.hasCargo;
                visionTarget = actionCargo && sense.hasCargo;
            } else {
                cargoGather = false;
                releaseCargo = false;
                hatchGather = gather;
                releaseHatch = shoot;
                visionTarget = actionHatch;
            }
            setElevatorHeight();
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

    private void parseControlBoard(){
        if(controlBoard.getRawButton(cb.high)){
            rocketCargoState = RocketCargoshipPosition.HI;
            if (shift) setElevatorHeight();
        }else if(controlBoard.getRawButton(cb.middle)){
            rocketCargoState = RocketCargoshipPosition.MID;
            if (shift) setElevatorHeight();
        }else if(controlBoard.getRawButton(cb.low)){
            rocketCargoState = RocketCargoshipPosition.LO;
            if (shift) setElevatorHeight();
        }else if(controlBoard.getRawButton(cb.front)){
            rocketCargoState = RocketCargoshipPosition.FRONT;
            if (shift) setElevatorHeight();
        }

        switch(rocketCargoState){
            case HI:
            controlBoard.setOutput(cb.high, false);
            controlBoard.setOutput(cb.middle, true);
            controlBoard.setOutput(cb.low, true);
            controlBoard.setOutput(cb.front, true);
            break;

            case MID:
            controlBoard.setOutput(cb.high, true);
            controlBoard.setOutput(cb.middle, false);
            controlBoard.setOutput(cb.low, true);
            controlBoard.setOutput(cb.front, true);
            break;

            case LO:
            controlBoard.setOutput(cb.high, true);
            controlBoard.setOutput(cb.middle, true);
            controlBoard.setOutput(cb.low, false);
            controlBoard.setOutput(cb.front, true);
            break;

            case FRONT:
            controlBoard.setOutput(cb.high, true);
            controlBoard.setOutput(cb.middle, true);
            controlBoard.setOutput(cb.low, true);
            controlBoard.setOutput(cb.front, false );
            break;   
            
            case DEFAULT:
            controlBoard.setOutput(cb.high, false);
            controlBoard.setOutput(cb.middle, false);
            controlBoard.setOutput(cb.low, false);
            controlBoard.setOutput(cb.front, false);
            break;
        }
        

        if(controlBoard.getRawButton(cb.farRkt)){
            nearFarCargo = NearFarCargo.FAR;
            if (shift) setElevatorHeight();
        }else if(controlBoard.getRawButton(cb.nearRkt)){
            nearFarCargo = NearFarCargo.NEAR;
            if (shift) setElevatorHeight();
        }else if(controlBoard.getRawButton(cb.cargoShip)){
            nearFarCargo = NearFarCargo.CARGO;
            if (shift) setElevatorHeight();
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

        SmartDashboard.putString("RocketState",rocketCargoState.name());
        SmartDashboard.putString("NearFarCargoState",nearFarCargo.name());
    }

    public enum AutoGatherStates { DRIVETOGATHER, CAMERAGATHER };
    private AutoGatherStates autoGatherState = AutoGatherStates.DRIVETOGATHER;
    
    public void autoGather(boolean autoGather) {
        
        if(autoGather){
            enableCamera = true;

            switch(autoGatherState){
                case DRIVETOGATHER:
                //pathfind
                //face the robot in the right direction
                //move elevator to floor/feeder station
                if (cargoNotHatch){
                    elevatorTarget = ElevatorPosition.FLOOR;
                } else {
                    elevatorTarget = ElevatorPosition.LOADING_STATION;
                    camLightsOn = true;
                }
    
                //move gather to cargo/hatch position
                //this is handled in the hatchGather class

                //when camera sees target
                //when operator gather button pressed
                if(cargoNotHatch && view.goodCargoImage() || !cargoNotHatch && view.goodVisionTarget() || gather){
                    //next state
                    autoGatherState = AutoGatherStates.CAMERAGATHER;
                }
                break;

                case CAMERAGATHER:
                //camera drive, or manual drive if no image
                visionCargo = cargoNotHatch && view.goodCargoImage();
                visionTarget =  !cargoNotHatch && view.goodVisionTarget();
                
                //set elevator position
                if (cargoNotHatch){
                    elevatorTarget = ElevatorPosition.FLOOR;
                } else {
                    elevatorTarget = ElevatorPosition.LOADING_STATION;
                    camLightsOn = true;
                }

                //turn on gatherer
                hatchGather = !cargoNotHatch;
                cargoGather = cargoNotHatch;

                //when gather current spike
                if(sense.pdp.getCurrent(ElectroJendz.GTH_MotorL_ID) > k.GTH_CurrLimit){
                    //set sense.hasThing to true
                    //next state
                    sense.hasCargo = cargoNotHatch;
                    sense.hasHatch = !cargoNotHatch;
                    autoGatherState = AutoGatherStates.DRIVETOGATHER;
                    rocketCargoState = RocketCargoshipPosition.DEFAULT;
                    nearFarCargo = NearFarCargo.DEFAULT;
                }
                
                break;
            }

        } else { //reset to first state
            autoGatherState = AutoGatherStates.DRIVETOGATHER;
        }

    } 

    public enum AutoScoreStates { DRIVETOGOAL, CAMERASCORE, GATHERSHOOT }
    private AutoScoreStates autoScoreState = AutoScoreStates.DRIVETOGOAL;
    double autoScoreTimer = 0;

    public void autoScore(boolean autoScore) {

        if(autoScore){
            enableCamera = true;
            camLightsOn = true;

            switch(autoScoreState){
                case DRIVETOGOAL:
                //pathfind
                //face robot in right direction
                //move elevator to stage height (but dont go too high, wait at level 2 if target is level 3)
                setElevatorHeight();
                elevatorStage = true;

                //when vision spotted
                //when shift shoot
                if(cargoNotHatch && view.goodCargoImage() 
                        || !cargoNotHatch && view.goodVisionTarget() 
                        || shift && shoot){
                    //next state
                    autoScoreState = AutoScoreStates.CAMERASCORE;
                }
                
                break;

                case CAMERASCORE:
                //camera drive or manual drive if no image
                visionTarget =  view.goodVisionTarget();

                //move elevator to final height
                setElevatorHeight();
                
                //when reached target
                //when operator shoot button pressed (or when shift released)
                VisionData vd = view.getLastVisionTarget();
                if(vd != null && view.goodVisionTarget() && vd.distance < k.CAM_ShootDist || !shift && shoot){
                    //next state
                    autoScoreState = AutoScoreStates.GATHERSHOOT;
                    autoScoreTimer = Timer.getFPGATimestamp();
                }
                

                //when shoot released and no image
                if(!shoot && !view.goodVisionTarget()){
                    //go back to DRIVETOGOAL state
                    autoScoreState = AutoScoreStates.DRIVETOGOAL;
                }
                
                break;

                case GATHERSHOOT:

                //run shoot gather
                releaseCargo = cargoNotHatch;
                releaseHatch = !cargoNotHatch;
                
                //when TIME_CAL elapses (like 0.5sec or so)
                if(Timer.getFPGATimestamp() - autoScoreTimer > k.GTH_ReleaseTime){
                    //turn off gather
                    //next state
                    autoScoreState = AutoScoreStates.DRIVETOGOAL;
                    sense.hasCargo = false;
                    sense.hasHatch = false;
                }
                
                break;
            }

        } else { //reset to first state
            autoScoreState = AutoScoreStates.DRIVETOGOAL;
        }

    }

    //set elevator position based on control board state
    public void setElevatorHeight(){
        switch(nearFarCargo){
            case NEAR:
            case FAR:
                switch(rocketCargoState){
                    case HI:
                        if(cargoNotHatch){
                            elevatorTarget = ElevatorPosition.ROCKET_3_CARGO;
                        } else {
                            elevatorTarget = ElevatorPosition.ROCKET_3_HATCH;
                        }
                    break;

                    case MID:
                        if(cargoNotHatch){
                            elevatorTarget = ElevatorPosition.ROCKET_2_CARGO;
                            
                        } else {
                            elevatorTarget = ElevatorPosition.ROCKET_2_HATCH;
                        }
                    break;

                    case LO:
                        if(cargoNotHatch){
                            elevatorTarget = ElevatorPosition.ROCKET_1_CARGO;
                        } else {
                            elevatorTarget = ElevatorPosition.ROCKET_1_HATCH;
                        }
                    break;

                    case DEFAULT:
                        elevatorTarget = ElevatorPosition.DONT_MOVE;
                    break;

                    case FRONT:
                        if(cargoNotHatch){
                            elevatorTarget = ElevatorPosition.FLOOR;
                        } else {
                            elevatorTarget = ElevatorPosition.LOADING_STATION;
                        }
                    break;
                }
            break;

            case CARGO:
                if(cargoNotHatch){
                    elevatorTarget = ElevatorPosition.SHIP_CARGO;
                } else {
                    elevatorTarget = ElevatorPosition.SHIP_HATCH;
                }
            break;

            case DEFAULT:
                elevatorTarget = ElevatorPosition.DONT_MOVE;
            break;
        }
    }

}