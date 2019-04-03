package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.io.ElectroJendz;
import frc.robot.io.ControlBoard.NearFarCargo;
import frc.robot.util.Util;

public class ScorpioGatherer extends ArmGatherer {

    private enum ScorpioState {
        INIT, WAIT_FOR_START, MOVING_TO_LIMIT, TAKING_ACTION, RETRACT_ARM
    }
    private ScorpioState armState = ScorpioState.INIT;

    private enum ActionState {
        WAIT, GATHER_CARGO, GATHER_HATCH, SHOOT_CARGO_CSHIP, SHOOT_CARGO_ROCKET, SHOOT_HATCH
    }
    private ActionState actionState = ActionState.WAIT;
    
    private double actionTimer;

    public ScorpioGatherer(){

    }

    @Override
    public void run(){
        if(k.GTH_DisableGather) return;

        if(!in.autoNotManualMode){
            if(in.controlBoard.jogUp){
                out.setGatherArm(0.5);
            } else if(in.controlBoard.jogDown){
                out.setGatherArm(-0.5);
            } else {
                out.setGatherArm(0);
            }

            if(in.gather) out.setGatherWheels(0.5);
            else if(in.shoot) out.setGatherWheels(-0.5);
            else out.setGatherWheels(0);
        }
        else { //automatic mode
            
            double wheelHoldPower = 0;
            if(sense.hasHatch) wheelHoldPower = k.SCR_HatchHoldPwr;
            else if(sense.hasCargo) wheelHoldPower = k.SCR_CargoHoldPwr;

            double wheelPower = 0;
            double extendTime = 0;
            double dwellTime = 0;
            switch(actionState){
                case WAIT:
                    wheelPower = 0;
                    extendTime = 0;
                    dwellTime = 0;
                break;
                case GATHER_HATCH:
                    wheelPower = k.SCR_HatchGatherPwr;
                    extendTime = k.SCR_FullExtendTime;
                    dwellTime = k.SCR_HatchGatherTime;
                break;
                case GATHER_CARGO:
                    wheelPower = k.SCR_CargoGatherPwr;
                    extendTime = k.SCR_FullExtendTime;
                    dwellTime = k.SCR_CargoGatherTime;
                break;
                case SHOOT_CARGO_CSHIP:
                    wheelPower = k.SCR_CargoShootPwrCShip;
                    extendTime = k.SCR_HalfExtendTime;
                    dwellTime = k.SCR_CargoShootTime;
                break;
                case SHOOT_CARGO_ROCKET:
                    wheelPower = k.SCR_CargoShootPwrRocket;
                    extendTime = k.SCR_ShortExtendTime; 
                    dwellTime = k.SCR_CargoShootTime;
                break;
                case SHOOT_HATCH:
                    wheelPower = k.SCR_HatchShootPwr;
                    extendTime = k.SCR_FullExtendTime;
                    dwellTime = k.SCR_HatchShootTime;
                break;
            }
            
            switch(armState){
                case INIT:

                    actionState = ActionState.WAIT;
                    //determine what the request is
                    if(in.gatherHatch) actionState = ActionState.GATHER_HATCH;
                    else if(in.gatherCargo) actionState = ActionState.GATHER_CARGO;
                    else if(in.releaseHatch) actionState = ActionState.SHOOT_HATCH;
                    else if(in.releaseCargo) {
                        if(in.controlBoard.nearFarCargo == NearFarCargo.CARGO) actionState = ActionState.SHOOT_CARGO_CSHIP;
                        else actionState = ActionState.SHOOT_CARGO_ROCKET;
                    }

                    if(actionState != ActionState.WAIT) armState = ScorpioState.WAIT_FOR_START;

                    out.setGatherArm(k.SCR_ArmIdleHoldPower);
                    out.setGatherWheels(wheelHoldPower);

                break;

                case WAIT_FOR_START:

                    //wait until the elevator is ready
                    if(elevator.getElevatorError() < 3) {
                        armState = ScorpioState.MOVING_TO_LIMIT;
                        actionTimer = Timer.getFPGATimestamp() + extendTime;
                    }

                    out.setGatherArm(k.SCR_ArmIdleHoldPower);
                    out.setGatherWheels(wheelHoldPower);
                    
                break;

                case MOVING_TO_LIMIT:

                    //check for arm current, wheel current, time, encoder?
                    if(sense.pdp.getCurrent(ElectroJendz.GTH_ArmMotorID) > k.SCR_ArmOutCurrentLimit
                            || sense.pdp.getCurrent(ElectroJendz.GTH_MotorL_ID) > k.SCR_WheelCurrentLimit
                            || Timer.getFPGATimestamp() > actionTimer){
                        
                        armState = ScorpioState.TAKING_ACTION;
                        actionTimer = Timer.getFPGATimestamp() + dwellTime;
                    }

                    //if not gathering, continue to hold
                    if(actionState == ActionState.GATHER_CARGO || actionState == ActionState.GATHER_HATCH){
                        wheelPower = wheelHoldPower;
                    }

                    out.setGatherWheels(wheelPower);
                    out.setGatherArm(k.SCR_ArmOutPower);

                break;

                case TAKING_ACTION:

                    //wait for time
                    if(Timer.getFPGATimestamp() > actionTimer){
                        armState = ScorpioState.RETRACT_ARM;
                        actionTimer = Timer.getFPGATimestamp() + k.SCR_RetractTime;

                        //set sense.hasThing (if gathering and current spike, or shooting and not current spike)
                        switch(actionState){
                            case WAIT:

                            break;
                            case GATHER_CARGO:
                                if(checkWheelCurrent()) sense.hasCargo = true;
                            break;
                            case GATHER_HATCH:
                                if(checkWheelCurrent()) sense.hasHatch = true;
                            break;
                            case SHOOT_CARGO_CSHIP:
                            case SHOOT_CARGO_ROCKET:
                                sense.hasCargo = false;
                            break;
                            case SHOOT_HATCH:
                                sense.hasHatch = false;
                            break;
                        }
                    }

                    out.setGatherWheels(wheelPower);
                    out.setGatherArm(0);
                    
                break;

                case RETRACT_ARM:

                    //wait for time or in current
                    if(sense.pdp.getCurrent(ElectroJendz.GTH_ArmMotorID) > k.SCR_ArmInCurrentLimit
                            || Timer.getFPGATimestamp() > actionTimer){
                        armState = ScorpioState.INIT;
                    }

                    out.setGatherWheels(wheelHoldPower);
                    out.setGatherArm(k.SCR_ArmInPower);
                    
                break;
            }
        }
    }

    public boolean checkWheelCurrent(){
        return sense.pdp.getCurrent(ElectroJendz.GTH_MotorL_ID) > k.SCR_WheelCurrentLimit;
    }

}