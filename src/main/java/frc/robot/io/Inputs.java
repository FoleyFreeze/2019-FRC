package frc.robot.io;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Component;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class Inputs extends Component {
   
    public ControlBoard controlBoard;
    public Controller gamePad;
        
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
    public boolean gatherCargo;
    public boolean releaseCargo; 
    public boolean gatherHatch;
    public boolean releaseHatch; 

    public boolean climb;
    private boolean prevClimb;
    public boolean fallingEdgeClimb;
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
    private double gatherTimer; 

    public boolean leftNotRight;
    public boolean autoNotManualMode;

    public boolean fourBarIn;
    public boolean fourBarOut;

    public boolean visionCargo;
    public boolean visionTargetHigh;
    public boolean visionTargetLow;
    public boolean searchingCargo;
    public boolean scoringCargo;
    public boolean searchingHatch;

    public boolean autoDrive;
    private boolean prevAutoDrive;
    public boolean autoDriveRising;

    public boolean actionLeft;
    public boolean prevActionLeft; 
    public boolean actionLeftRising;
    public boolean actionLeftFalling;
    public boolean actionRight;
    public boolean camLightsOn;
    public boolean enableCamera; 

    public double robotOrientation;
    public boolean autoOrientRobot;

    public byte[] xDixon;//history of joystick directions
    public byte[] yDixon;//history of joystick directions
    public byte[] rotDixon;//history of joystick directions

  

    public Inputs() {
        controlBoard = new ControlBoard(ElectroJendz.CONTROL_BOARD);
        gamePad = new Controller(ElectroJendz.GAMEPAD);
        xDixon = new byte[k.IN_DixonSize];
        yDixon = new byte[k.IN_DixonSize];
        rotDixon = new byte[k.IN_DixonSize];
        controlBoard.nearFarCargo = ControlBoard.NearFarCargo.DEFAULT;
        controlBoard.rocketCargoState = ControlBoard.RocketCargoshipPosition.DEFAULT;
        elevatorTarget = ElevatorPosition.DONT_MOVE;
    }

    

    public void run() {
        
        gamePad.read();
        controlBoard.read();

        resetGyro = gamePad.resetGyro;
        if(resetGyro){
            sense.init();
        }

        resetEleEnc = gamePad.resetElePart1 && gamePad.resetElePart2;
        if(resetEleEnc){
            out.resetEleEnc();
        }
     
        //read joystick
        xAxisDrive = -gamePad.xDriveAxis;
        yAxisDrive = gamePad.yDriveAxis;
        rotAxisDrive = gamePad.rotDriveAxis;

        //deadband if tiny
        if(Math.abs(xAxisDrive) < gamePad.IN_xDeadband) xAxisDrive = 0; 
        if(Math.abs(yAxisDrive) < gamePad.IN_xDeadband) yAxisDrive = 0; 
        if(Math.abs(xAxisDrive) < gamePad.IN_xyDeadband && Math.abs(yAxisDrive) < gamePad.IN_xyDeadband) {
            xAxisDrive = 0;
            yAxisDrive = 0;
        }
        if(Math.abs(rotAxisDrive) < gamePad.IN_rotDeadband) rotAxisDrive = 0;//if command is unreasonably tiny, don't turn
        xAxisDrive = Math.signum(xAxisDrive) * (Math.abs(xAxisDrive) - gamePad.IN_xDeadband / (1-gamePad.IN_xDeadband));
        yAxisDrive = Math.signum(yAxisDrive) * (Math.abs(yAxisDrive) - gamePad.IN_yDeadband / (1-gamePad.IN_yDeadband));
        rotAxisDrive = Math.signum(rotAxisDrive) * (Math.abs(rotAxisDrive) - gamePad.IN_rotDeadband / (1-gamePad.IN_rotDeadband));

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

        //add joystick expo
        xAxisDrive = Math.signum(xAxisDrive) * Math.pow(Math.abs(xAxisDrive), k.DRV_AxisExpo);
        yAxisDrive = Math.signum(yAxisDrive) * Math.pow(Math.abs(yAxisDrive), k.DRV_AxisExpo);
        rotAxisDrive = Math.signum(rotAxisDrive) * Math.pow(Math.abs(rotAxisDrive), k.DRV_AxisExpo);

        cargoNotHatch = !controlBoard.cargoOrHatch;
        leftNotRight = !controlBoard.lOrR;
        autoNotManualMode = controlBoard.autoOrSemi;

        
        //set buttons
        if(!k.DRV_Disable){
            compassDrive = gamePad.compassDrive;
            fieldOriented = gamePad.fieldOriented && sense.navxGood;
            //dodgingL = gamePad.getRawAxis(k.IN_dodgingL) > k.IN_DodgingMin;
            //dodgingR = gamePad.getRawAxis(k.IN_dodgingR) > k.IN_DodgingMin;
            //visionCargo = gamePad.getRawButton(4);
        }
        //actionCargo = gamePad.getRawAxis(k.IN_dodgingL) > k.IN_DodgingMin;
        //actionHatch = gamePad.getRawAxis(k.IN_dodgingR) > k.IN_DodgingMin;
        actionLeft = gamePad.leftTrigger > k.IN_DodgingMin;
        actionLeftRising = actionLeft && !prevActionLeft;
        actionLeftFalling = !actionLeft && prevActionLeft;
        prevActionLeft = actionLeft;
        actionRight = gamePad.rightTrigger > k.IN_DodgingMin;

        //flipOrientation = gamePad.getRawButton(k.IN_flipOrientation);
        pitMode = !controlBoard.pitMode;
        shift = controlBoard.shift;
        shoot = controlBoard.shoot;
        if(sense.isDisabled) gatherTimer = Timer.getFPGATimestamp() + k.GTH_StartTimer;
        gather = controlBoard.gather && Timer.getFPGATimestamp() > gatherTimer;
        SmartDashboard.putBoolean("Pit Mode", pitMode);

        SmartDashboard.putBoolean("ActionLeft", actionLeft);
        SmartDashboard.putBoolean("ActionRight", actionRight);  

        autoElevator = true;
        //releaseCargo = false;
        //releaseHatch = false;
        //gatherHatch = false;
        //gatherCargo = false;
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
            boolean temp = controlBoard.climb;
            climb = temp && !shift;
            reverseClimb = (temp && shift);
            if(climb) cargoNotHatch = false;
            fallingEdgeClimb = prevClimb && !climb;
            prevClimb = climb;
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

        double remainingShootTime = Timer.getFPGATimestamp() - autoShootTime;

        if(autoNotManualMode){
            if(cargoNotHatch) {
                gatherHatch = false;
                releaseHatch = false; 
                gatherCargo = !sense.hasCargo && actionLeft && !releaseCargo || gather && !shift;
                releaseCargo = releaseCargo && !actionLeftFalling || sense.hasCargo && actionLeftRising;  
                
                if(!sense.hasCargo && shift && gather) {
                    sense.hasCargo = true;
                    sense.hasHatch = false;

                // After pressing shoot, reset no matter state of button holding
                } else if(sense.hasCargo && releaseCargo || sense.hasCargo && remainingShootTime<0 && remainingShootTime>-0.1) {
                    if(actionLeftRising) autoShootTime = Timer.getFPGATimestamp() + k.GTH_ReleaseTime;
                    else if(remainingShootTime<0) sense.hasCargo = false; 
                } else if(sense.pdp.getCurrent(ElectroJendz.GTH_MotorL_ID) > k.GTH_CurrLimit && gatherCargo) {
                    sense.hasCargo = true;
                    sense.hasHatch = false;
                }
            } else {// auto hatch
                gatherCargo = false; 
                releaseCargo = false; 
                gatherHatch = !sense.hasHatch && actionLeft && !releaseHatch || gather && !shift;
                releaseHatch = releaseHatch && !actionLeftFalling || sense.hasHatch && actionLeftRising; 
                //the overide
                if(!sense.hasHatch && shift && gather) {
                    sense.hasHatch = true;
                    sense.hasCargo = false;
                // After pressing shoot, reset no matter state of button holding
                } else if(sense.hasHatch && releaseHatch || sense.hasHatch && remainingShootTime<0 && remainingShootTime>-0.1) {
                    if(actionLeftRising) autoShootTime = Timer.getFPGATimestamp() + k.GTH_ReleaseTime;
                    else if(remainingShootTime<0) sense.hasHatch = false; 
                } else if(sense.pdp.getCurrent(ElectroJendz.GTH_MotorL_ID) > k.GTH_CurrLimit && gatherHatch) {
                    sense.hasHatch = true;
                    sense.hasCargo = false;
                }
            } 

            //autoDrive logic
            autoDrive = gamePad.autoDrive && actionRight;
            autoDriveRising = autoDrive && !prevAutoDrive;
            prevAutoDrive = autoDrive;

            // if ready
            if(actionRight || cargoNotHatch && sense.hasCargo && shoot || !cargoNotHatch && sense.hasHatch && shoot) {
                //only lift the elevator when autoDrive is not running or has finished its path
                setElevatorHeight(!autoDrive || autoDrive && autoDriving.pathComplete);
                setRobotOrientation();
                autoOrientRobot = true;
            } else {
                //not ready, so keep elevator down
                setElevatorHeight(false);
            }

            visionTargetLow = !cargoNotHatch && actionRight && gamePad.camDrive;
            visionTargetHigh = cargoNotHatch && actionRight && sense.hasCargo && gamePad.camDrive;
            visionCargo = cargoNotHatch && actionRight && !sense.hasCargo && gamePad.camDrive;

        } else { //manual mode
            if (cargoNotHatch){
                gatherCargo = gather && !shift;
                releaseCargo = shoot;
                gatherHatch = false;
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

                visionCargo = actionLeft && !sense.hasCargo;
                visionTargetHigh = actionLeft && sense.hasCargo;
            } else {// manual hatch
                gatherCargo = false;
                releaseCargo = false;
                gatherHatch = gather && !shift;
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

                visionTargetLow = actionRight;
            }
            
            //in manual mode we are always "ready" (no preloading elevator positions)
            setElevatorHeight(true);

            autoDrive = false;
            autoDriveRising = false;
            prevAutoDrive = false;
        }

        searchingCargo = !sense.isDisabled && cargoNotHatch && !sense.hasCargo;
        scoringCargo = !sense.isDisabled && cargoNotHatch && sense.hasCargo;
        searchingHatch = !sense.isDisabled && !cargoNotHatch;
        boolean targetAnything = searchingCargo || scoringCargo || searchingHatch; 
        enableCamera = targetAnything || k.CAM_DebugCargo || k.CAM_DebugTargetHigh || k.CAM_DebugTargetLow;
        camLightsOn = scoringCargo || searchingHatch || k.CAM_DebugTargetHigh || k.CAM_DebugTargetLow; //no lights for cargo targeting
        
        //handle sense.hasThing rising and falling edges
        sense.hasHatchEdge = sense.hasHatch != sense.prevHasHatch;
        sense.prevHasHatch = sense.hasHatch;
    }

    private double autoShootTime;

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
    
    //set elevator position based on control board state
    public void setElevatorHeight(boolean ready){
        if(climb) {
            //elevatorTarget = ElevatorPosition.ROCKET_1_HATCH;
            //return;
            controlBoard.rocketCargoState = ControlBoard.RocketCargoshipPosition.LO;
            controlBoard.nearFarCargo = ControlBoard.NearFarCargo.NEAR;
        }

        //if not ready, dont lift the elevator yet
        if(!ready){
            if(!cargoNotHatch || sense.hasCargo) {
                elevatorTarget = ElevatorPosition.LOADING_STATION;
            } else {
                elevatorTarget = ElevatorPosition.FLOOR;
            }
        }

        switch(controlBoard.nearFarCargo){
            case NEAR:
            case FAR:
                switch(controlBoard.rocketCargoState){
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
                            //elevatorTarget = ElevatorPosition.LOADING_STATION;
                            elevatorTarget = ElevatorPosition.FLOOR;
                        }
                    break;
                }
            break;

            case CARGO:
                switch(controlBoard.rocketCargoState){
                    case HI:
                    case MID:
                    case LO:
                    case FRONT:
                        if(cargoNotHatch){
                            elevatorTarget = ElevatorPosition.SHIP_CARGO;
                        } else {
                            elevatorTarget = ElevatorPosition.SHIP_HATCH;
                        }
                        break;

                    /*case FRONT:
                        elevatorTarget = ElevatorPosition.FLOOR;
                        break;*/

                    case DEFAULT:
                        elevatorTarget = ElevatorPosition.DONT_MOVE;
                        break;
                }
                
            break;

            case DEFAULT:
                elevatorTarget = ElevatorPosition.DONT_MOVE;
            break;
        }
    }

    private void setRobotOrientation(){

        switch(controlBoard.nearFarCargo){
            case NEAR: 
                if(cargoNotHatch) {
                    robotOrientation = 90;
                } else {
                    robotOrientation = 29;
                }   
            break; 

            case FAR: 
                if(cargoNotHatch) {
                    robotOrientation = 90;
                } else {
                    robotOrientation = 149;
                } 
            break; 

            case CARGO: 
                switch(controlBoard.rocketCargoState) {
                    case LO: 
                    case MID: 
                    case HI: 
                        robotOrientation = -90;
                    break;

                    case FRONT: 
                        robotOrientation = 0;
                    break;

                    case DEFAULT: 
                        robotOrientation = 910;
                    break;
                }
            break;

            case DEFAULT:
                robotOrientation = 910;
            break;
        }   
        if(!leftNotRight) robotOrientation = -robotOrientation;
    }
}