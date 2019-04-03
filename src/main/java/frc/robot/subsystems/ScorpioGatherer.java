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

    private double targetPosition;

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
                        if(actionState == ActionState.SHOOT_CARGO_CSHIP){
                            targetPosition = k.SCR_PartOutPosition;
                        } else {
                            targetPosition = k.SCR_FullOutPosition;
                        }
                    }

                    out.setGatherArm(k.SCR_ArmIdleHoldPower);
                    out.setGatherWheels(wheelHoldPower);
                    
                break;

                case MOVING_TO_LIMIT:

                    //check for arm current, encoder value, time
                    if(sense.pdp.getCurrent(ElectroJendz.GTH_ArmMotorID) > k.SCR_ArmOutCurrentLimit
                            || checkArmPosition()
                            || Timer.getFPGATimestamp() > actionTimer){
                        
                        armState = ScorpioState.TAKING_ACTION;
                        actionTimer = Timer.getFPGATimestamp() + dwellTime;
                    }

                    //if not gathering, continue to hold
                    if(actionState == ActionState.GATHER_CARGO || actionState == ActionState.GATHER_HATCH){
                        wheelPower = wheelHoldPower;
                    }

                    out.setGatherWheels(wheelPower);
                    //out.setGatherArm(k.SCR_ArmOutPower);
                    pidArm(targetPosition);

                break;

                case TAKING_ACTION:

                    //if gathering, leave scorpio extended until either the gather button is released or we see the current spike
                    if(actionState == ActionState.GATHER_CARGO){
                        if(checkWheelCurrent()){
                            sense.hasCargo = true;
                            armState = ScorpioState.RETRACT_ARM;
                            actionTimer = Timer.getFPGATimestamp() + k.SCR_RetractTime;
                        } else if(!in.gatherCargo){
                            armState = ScorpioState.RETRACT_ARM;
                            actionTimer = Timer.getFPGATimestamp() + k.SCR_RetractTime;
                        }
                    } else if(actionState == ActionState.GATHER_HATCH){
                        if(checkWheelCurrent()){
                            sense.hasHatch = true;
                            armState = ScorpioState.RETRACT_ARM;
                            actionTimer = Timer.getFPGATimestamp() + k.SCR_RetractTime;
                        } else if(!in.gatherHatch){
                            armState = ScorpioState.RETRACT_ARM;
                            actionTimer = Timer.getFPGATimestamp() + k.SCR_RetractTime;
                        }

                    } else {
                        //if shooting, shoot for time
                        if(Timer.getFPGATimestamp() > actionTimer){
                            armState = ScorpioState.RETRACT_ARM;
                            actionTimer = Timer.getFPGATimestamp() + k.SCR_RetractTime;

                            sense.hasHatch = false;
                            sense.hasCargo = false;
                        }    
                    }

                    out.setGatherWheels(wheelPower);
                    out.setGatherArm(0); //do we want to hold it in position?
                    
                break;

                case RETRACT_ARM:

                    //wait for time or in current
                    if(sense.pdp.getCurrent(ElectroJendz.GTH_ArmMotorID) > k.SCR_ArmInCurrentLimit
                            || checkArmPosition()
                            || Timer.getFPGATimestamp() > actionTimer){
                        armState = ScorpioState.INIT;
                    }

                    targetPosition = k.SCR_InPosition;
                    out.setGatherWheels(wheelHoldPower);
                    //out.setGatherArm(k.SCR_ArmInPower);
                    pidArm(targetPosition);
                    
                break;
            }
        }
    }

    private boolean checkWheelCurrent(){
        return sense.pdp.getCurrent(ElectroJendz.GTH_MotorL_ID) > k.SCR_WheelCurrentLimit;
    }

    private void pidArm(double position){

        double error = position - sense.scorpioArmEnc;
        double power = error * k.SCR_ArmPositionKP;
        power = Util.limit(power, k.SCR_ArmPowerLimit);

        out.setGatherArm(power);
    }

    private boolean checkArmPosition(){
        return Math.abs(sense.scorpioArmEnc - targetPosition) < 1;
    }

}