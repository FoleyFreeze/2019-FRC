package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.ControlBoard.NearFarCargo;

public class ArmGatherer extends Component{

    private enum FourBarState{
        WAIT, STARTUP, WAIT_FOR_STALL
    }
    private FourBarState gatherState = FourBarState.WAIT;

    public ArmGatherer() {
        
    }

    private double startTime;
    private boolean movingToCargo;
    private boolean inCargoPosition;
    private boolean inHatchPosition;

    private double gatherTimer;
    private double shootTimer;
    private boolean gatherTimerActive;
    private boolean shootTimerActive;

    public void run() {
        if(k.GTH_DisableGather) return;

        //GATHER ARM
       
        //make sure that if we disable in a movement we don't re-enable and immediately start moving
        if(sense.isDisabled) gatherState = FourBarState.WAIT;
       
        //out.suction(in.hatchGather);
        
        SmartDashboard.putString("GatherArmState", gatherState.name());

        switch(gatherState){
            case WAIT:
            if(in.cargoNotHatch){
                out.setGatherArm(0);
            } else {
                //help keep it closed
                out.setGatherArm(k.GTH_ArmExtraIdlePwr);
            }

            if(in.cargoNotHatch && !sense.hasHatch && !inCargoPosition){
                movingToCargo = true;
                startTime = Timer.getFPGATimestamp();
                gatherState = FourBarState.STARTUP;    
            } else if(!in.cargoNotHatch && !sense.hasCargo && !inHatchPosition){
                movingToCargo = false;
                startTime = Timer.getFPGATimestamp();
                gatherState = FourBarState.STARTUP;
            }
            break;

            case STARTUP:
            if(movingToCargo) out.setGatherArm(-k.GTH_Arm2CargoPwr);
            else out.setGatherArm(k.GTH_Arm2HatchPwr);

            if(Timer.getFPGATimestamp() - startTime > k.GTH_StartUpTime){
                gatherState = FourBarState.WAIT_FOR_STALL;
                startTime = Timer.getFPGATimestamp();
            }
            break;

            case WAIT_FOR_STALL:
            if(movingToCargo) out.setGatherArm(-k.GTH_Arm2CargoPwr);
            else out.setGatherArm(k.GTH_Arm2HatchPwr);

            if(movingToCargo && out.getGatherArmCurrent() > k.GTH_ArmOutCurrent
                    || Timer.getFPGATimestamp() - startTime > k.GTH_Arm2CargoTimer){
                gatherState = FourBarState.WAIT;
                inCargoPosition = true;
                inHatchPosition = false;
            }else if(!movingToCargo && out.getGatherArmCurrent() > k.GTH_ArmInCurrent
                    || Timer.getFPGATimestamp() - startTime > k.GTH_Arm2HatchTimer){
                gatherState = FourBarState.WAIT;
                inCargoPosition = false;
                inHatchPosition = true;
            }
            break;
            
        }

        //GATHER WHEELS
        
        // conditions for gathering
        if(in.gatherCargo){
            if(!gatherTimerActive){
                gatherTimerActive = true;
                gatherTimer = Timer.getFPGATimestamp() + k.GTH_CargoGatherTime;
            }
            out.setGatherMotor(k.GTH_CargoIntakeSpeed, -k.GTH_CargoIntakeSpeed);
        }
        //  releases cargo
        else if(in.releaseCargo){
                if(!shootTimerActive){
                    shootTimerActive = true;
                    shootTimer = Timer.getFPGATimestamp() + k.GTH_CargoShootTime;
                }
                if (in.controlBoard.nearFarCargo == NearFarCargo.CARGO) {
                    out.setGatherMotor(-k.GTH_CargoShootSpeedFast, k.GTH_CargoShootSpeedSlow);
                } else {
                    out.setGatherMotor(-k.GTH_CargoShootSpeedFast, k.GTH_CargoShootSpeedSlow);
                }
        }
        else if(in.gatherHatch){
            if(!gatherTimerActive){
                gatherTimerActive = true;
                gatherTimer = Timer.getFPGATimestamp() + k.GTH_HatchGatherTime;
            }
            out.setGatherMotor(-k.GTH_HatchIntakeSpeed, k.GTH_HatchIntakeSpeed);
        }
        //  releases hatch
        else if(in.releaseHatch){
            if(!shootTimerActive){
                shootTimerActive = true;
                shootTimer = Timer.getFPGATimestamp() + k.GTH_HatchShootTime;
            }
            out.setGatherMotor(k.GTH_HatchShootSpeedFast, -k.GTH_HatchShootSpeedSlow);
        }
        else { //no active gather/shoot request
            
            gatherTimerActive = false;
            shootTimerActive = false;

            if(sense.hasCargo){
                out.setGatherMotor(k.GTH_CargoHoldSpeed, -k.GTH_CargoHoldSpeed);
            }
            else if(sense.hasHatch){
                out.setGatherMotor(-k.GTH_HatchHoldSpeed, k.GTH_HatchHoldSpeed);
            }
            // stop moving
            else {
                out.setGatherMotor(0,0); 
            }
        } 

    }


    public boolean cargoGatherComplete(){
        return Timer.getFPGATimestamp() - gatherTimer > 0;
    }
    public boolean cargoShootComplete(){
        return Timer.getFPGATimestamp() - shootTimer > 0;
    }
    public boolean hatchGatherComplete(){
        return Timer.getFPGATimestamp() - gatherTimer > 0;
    }
    public boolean hatchShootComplete(){
        return Timer.getFPGATimestamp() - shootTimer > 0;
    }

}