package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.ControlBoard.NearFarCargo;
import frc.robot.io.ElectroJendz;

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

    private double cargoGatherTimer;
    private double cargoShootTimer;
    private double hatchGatherTimer;
    private double hatchShootTimer;
    private boolean cargoGatherTimerActive;
    private boolean cargoShootTimerActive;
    private boolean hatchGatherTimerActive;
    private boolean hatchShootTimerActive;
    private boolean cargoGatherComplete;
    private boolean cargoShootComplete;
    private boolean hatchGatherComplete;
    private boolean hatchShootComplete;

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

        if(cargoGatherTimerActive) {
            cargoGatherTimerActive = Timer.getFPGATimestamp() <= cargoGatherTimer && in.autoNotManualMode;
            if(!cargoGatherTimerActive){ //falling edge
                cargoGatherComplete = true;
                cargoShootComplete = false;
                hatchGatherComplete = false;
                hatchShootComplete = false;
            }
        }
        if(cargoShootTimerActive) {
            cargoShootTimerActive = Timer.getFPGATimestamp() <= cargoShootTimer && in.autoNotManualMode;
            if(!cargoShootTimerActive){ //falling edge
                cargoGatherComplete = false;
                cargoShootComplete = true;
                hatchGatherComplete = false;
                hatchShootComplete = false;
            }
        }
        if(hatchGatherTimerActive) {
            hatchGatherTimerActive = Timer.getFPGATimestamp() <= hatchGatherTimer && in.autoNotManualMode;
            if(!cargoGatherTimerActive){ //falling edge
                hatchGatherComplete = false;
                cargoShootComplete = false;
                hatchGatherComplete = true;
                hatchShootComplete = false;
            }
        }
        if(hatchShootTimerActive) {
            hatchShootTimerActive = Timer.getFPGATimestamp() <= hatchShootTimer && in.autoNotManualMode;
            if(!hatchShootTimerActive){ //falling edge
                cargoGatherComplete = false;
                cargoShootComplete = false;
                hatchGatherComplete = false;
                hatchShootComplete = true;
            }
        }

        // conditions for gathering
        if(in.gatherCargo || cargoGatherTimerActive){
            if(!cargoGatherTimerActive){
                cargoGatherTimerActive = true;
                cargoGatherTimer = Timer.getFPGATimestamp() + k.GTH_CargoGatherTime;
            }
            out.setGatherMotor(k.GTH_CargoIntakeSpeed, -k.GTH_CargoIntakeSpeed);

            //do current sense
            if(sense.pdp.getCurrent(ElectroJendz.GTH_MotorL_ID) + sense.pdp.getCurrent(ElectroJendz.GTH_MotorR_ID) > k.GTH_CurrLimit*2){
                cargoGatherTimer = 0; //so next iter it will complete
                sense.hasCargo = true;
            }
        }
        //  releases cargo
        else if(in.releaseCargo || cargoShootTimerActive){
                if(!cargoShootTimerActive){
                    cargoShootTimerActive = true;
                    cargoShootTimer = Timer.getFPGATimestamp() + k.GTH_CargoShootTime;
                }
                if (in.controlBoard.nearFarCargo == NearFarCargo.CARGO) {
                    out.setGatherMotor(-k.GTH_CargoShootSpeedFast, k.GTH_CargoShootSpeedSlow);
                } else {
                    out.setGatherMotor(-k.GTH_CargoShootSpeedFast, k.GTH_CargoShootSpeedSlow);
                }
        }
        else if(in.gatherHatch || hatchGatherTimerActive){
            if(!hatchGatherTimerActive){
                hatchGatherTimerActive = true;
                hatchGatherTimer = Timer.getFPGATimestamp() + k.GTH_HatchGatherTime;
            }
            out.setGatherMotor(-k.GTH_HatchIntakeSpeed, k.GTH_HatchIntakeSpeed);

            //do current sense
            if(sense.pdp.getCurrent(ElectroJendz.GTH_MotorL_ID) + sense.pdp.getCurrent(ElectroJendz.GTH_MotorR_ID) > k.GTH_CurrLimit*2){
                hatchGatherTimer = 0;
                sense.hasHatch = true;
            }
        }
        //  releases hatch
        else if(in.releaseHatch){
            if(!hatchShootTimerActive){
                hatchShootTimerActive = true;
                hatchShootTimer = Timer.getFPGATimestamp() + k.GTH_HatchShootTime;
            }
            out.setGatherMotor(k.GTH_HatchShootSpeedFast, -k.GTH_HatchShootSpeedSlow);
        }
        else { //no active gather/shoot request

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
        return cargoGatherComplete && in.autoNotManualMode;
    }
    public boolean cargoShootComplete(){
        return cargoShootComplete && in.autoNotManualMode;
    }
    public boolean hatchGatherComplete(){
        return hatchGatherComplete && in.autoNotManualMode;
    }
    public boolean hatchShootComplete(){
        return hatchShootComplete && in.autoNotManualMode;
    }

    public boolean scorpioActive(){
        return false;
    }
}