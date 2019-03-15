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
    public boolean visionTargetHigh;
    public boolean visionTargetLow;

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

        cargoNotHatch = !controlBoard.getRawButton(cb.cargoOrHatch);
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
        //actionCargo = gamePad.getRawAxis(k.IN_dodgingL) > k.IN_DodgingMin;
        //actionHatch = gamePad.getRawAxis(k.IN_dodgingR) > k.IN_DodgingMin;
        actionCargo = gamePad.getRawAxis(bm.IN_dodgingL) > k.IN_DodgingMin;
        actionHatch = gamePad.getRawAxis(bm.IN_dodgingR) > k.IN_DodgingMin;

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
        visionTargetHigh = false;
        visionTargetLow = false;
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
            SmartDashboard.putBoolean("GoodVisionTarget",view.goodVisionTargetHigh());
        } else {
            //manual mode
            if (cargoNotHatch){
                cargoGather = gather && !shift;
                releaseCargo = shoot;
                hatchGather = false;
                releaseHatch = false;

                if(!sense.hasCargo && shift && gather) {
                    sense.hasCargo = true;
                    sense.hasHatch = false;
                } else if(sense.hasCargo && shoot) {
                    sense.hasCargo = false; 
                } else if(sense.pdp.getCurrent(ElectroJendz.GTH_MotorL_ID) > k.GTH_CurrLimit && gather) {
                    sense.hasCargo = true;
                    sense.hasHatch = false;
                }

                visionCargo = actionCargo && !sense.hasCargo;
                visionTargetHigh = actionCargo && sense.hasCargo;
            } else {
                cargoGather = false;
                releaseCargo = false;
                hatchGather = gather && !shift;
                releaseHatch = shoot;

                if(!sense.hasHatch && shift && gather) {
                    sense.hasHatch = true;
                    sense.hasCargo = false;
                } else if(sense.hasHatch && shoot) {
                    sense.hasHatch = false;
                } else if(sense.pdp.getCurrent(ElectroJendz.GTH_MotorL_ID) > k.GTH_CurrLimit && gather) {
                    sense.hasHatch = true;
                    sense.hasCargo = false;
                }

                visionTargetLow = actionHatch;
            }
            setElevatorHeight();
        }

        boolean searchingCargo = !sense.isDisabled && cargoNotHatch && !sense.hasCargo;
        boolean scoringCargo = !sense.isDisabled && cargoNotHatch && sense.hasCargo;
        boolean searchingHatch = !sense.isDisabled && !cargoNotHatch;
        boolean targetAnything = searchingCargo || scoringCargo || searchingHatch; 
        enableCamera = targetAnything || k.CAM_DebugCargo || k.CAM_DebugTargetHigh || k.CAM_DebugTargetLow;
        camLightsOn = scoringCargo || searchingHatch || k.CAM_DebugTargetHigh || k.CAM_DebugTargetLow; //no lights for cargo targeting
        
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

        controlBoard.setOutput(cb.shiftOut, shift);

        switch(rocketCargoState){
            case HI:
            controlBoard.setOutput(cb.highOut, true);
            controlBoard.setOutput(cb.middleOut, false);
            controlBoard.setOutput(cb.lowOut, false);
            controlBoard.setOutput(cb.frontOut, false);
            break;

            case MID:
            controlBoard.setOutput(cb.highOut, false);
            controlBoard.setOutput(cb.middleOut, true);
            controlBoard.setOutput(cb.lowOut, false);
            controlBoard.setOutput(cb.frontOut, false);
            break;

            case LO:
            controlBoard.setOutput(cb.highOut, false);
            controlBoard.setOutput(cb.middleOut, false);
            controlBoard.setOutput(cb.lowOut, true);
            controlBoard.setOutput(cb.frontOut, false);
            break;

            case FRONT:
            controlBoard.setOutput(cb.highOut, false);
            controlBoard.setOutput(cb.middleOut, false);
            controlBoard.setOutput(cb.lowOut, false);
            controlBoard.setOutput(cb.frontOut, true);
            break;   
            
            case DEFAULT:
            controlBoard.setOutput(cb.highOut, true);
            controlBoard.setOutput(cb.middleOut, true);
            controlBoard.setOutput(cb.lowOut, true);
            controlBoard.setOutput(cb.frontOut, true);
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
            controlBoard.setOutput(cb.farRktOut, true);
            controlBoard.setOutput(cb.nearRktOut, false);
            controlBoard.setOutput(cb.cargoShipOut, false);
            break;

            case NEAR:
            controlBoard.setOutput(cb.farRktOut, false);
            controlBoard.setOutput(cb.nearRktOut, true);
            controlBoard.setOutput(cb.cargoShipOut, false);
            break;

            case CARGO:
            controlBoard.setOutput(cb.farRktOut, false);
            controlBoard.setOutput(cb.nearRktOut, false);
            controlBoard.setOutput(cb.cargoShipOut, true);
            break;  

            case DEFAULT:
            controlBoard.setOutput(cb.farRktOut, true);
            controlBoard.setOutput(cb.nearRktOut, true);
            controlBoard.setOutput(cb.cargoShipOut, true);
            break;
        }

        SmartDashboard.putString("RocketState",rocketCargoState.name());
        SmartDashboard.putString("NearFarCargoState",nearFarCargo.name());
    }

    public enum AutoGatherStates { DRIVETOGATHER, CAMERAGATHER };
    private AutoGatherStates autoGatherState = AutoGatherStates.DRIVETOGATHER;
    
    public void autoGather(boolean autoGather) {
        if(autoGather) {
        
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
                    enableCamera = true;

                    //move gather to cargo/hatch position
                    //this is handled in the hatchGather class

                    //when camera sees target
                    //add transition for when auto drive is completed
                    boolean driveComplete = true;
                    if(cargoNotHatch && view.goodCargoImage() || !cargoNotHatch && view.goodVisionTargetHigh() || driveComplete){
                        //next state
                        autoGatherState = AutoGatherStates.CAMERAGATHER;
                    }
                break;

                case CAMERAGATHER:
                    //camera drive, or manual drive if no image
                    enableCamera = true;
                    visionCargo = cargoNotHatch && view.goodCargoImage();
                    visionTargetHigh =  !cargoNotHatch && view.goodVisionTargetHigh();
                    
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
                    if(sense.pdp.getCurrent(ElectroJendz.GTH_MotorL_ID) > k.GTH_CurrLimit || shift && gather){
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
                    //when drive complete
                    boolean driveComplete = true;
                    if(cargoNotHatch && view.goodCargoImage()
                            || !cargoNotHatch && view.goodVisionTargetHigh()
                            || driveComplete){
                        //next state
                        autoScoreState = AutoScoreStates.CAMERASCORE;
                    }
                break;

                case CAMERASCORE:
                    //camera drive or manual drive if no image
                    visionTargetHigh =  view.goodVisionTargetHigh();

                    //move elevator to final height
                    setElevatorHeight();
                    
                    //when reached target
                    //when operator shoot button pressed
                    VisionData vd = view.getLastVisionTargetHigh();
                    if(vd != null && view.goodVisionTargetHigh() && vd.distance < k.CAM_ShootDist || shoot){
                        //next state
                        autoScoreState = AutoScoreStates.GATHERSHOOT;
                        autoScoreTimer = Timer.getFPGATimestamp();
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