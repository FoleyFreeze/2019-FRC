package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HatchGatherer extends Component{

    private enum FourBarState{
        WAIT, STARTUP, WAIT_FOR_STALL
    }
    private FourBarState gatherState = FourBarState.WAIT;

    public HatchGatherer() {
        
    }

    private double startTime;
    private boolean movingToBall;
    private boolean inBallPosition;
    private boolean inHatchPosition;

    public void run() {
        if(k.GTH_DisableHatch) return;
       
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
                out.setGatherArm(0.05);
            }

            if(in.cargoNotHatch && !sense.hasHatch && !inBallPosition){
                movingToBall = true;
                startTime = Timer.getFPGATimestamp();
                gatherState = FourBarState.STARTUP;    
            } else if(!in.cargoNotHatch && !sense.hasCargo && !inHatchPosition){
                movingToBall = false;
                startTime = Timer.getFPGATimestamp();
                gatherState = FourBarState.STARTUP;
            }
            break;

            case STARTUP:
            if(movingToBall) out.setGatherArm(-k.GTH_Arm2CargoPwr);
            else out.setGatherArm(k.GTH_Arm2HatchPwr);

            if(Timer.getFPGATimestamp() - startTime > k.GTH_StartUpTime){
                gatherState = FourBarState.WAIT_FOR_STALL;
                startTime = Timer.getFPGATimestamp();
            }
            break;

            case WAIT_FOR_STALL:
            if(movingToBall) out.setGatherArm(-k.GTH_Arm2CargoPwr);
            else out.setGatherArm(k.GTH_Arm2HatchPwr);

            if(movingToBall && out.getGatherArmCurrent() > k.GTH_ArmInCurrent
                    || Timer.getFPGATimestamp() - startTime > k.GTH_Arm2CargoTimer){
                gatherState = FourBarState.WAIT;
                inBallPosition = true;
                inHatchPosition = false;
            }else if(!movingToBall && out.getGatherArmCurrent() > k.GTH_ArmOutCurrent
                    || Timer.getFPGATimestamp() - startTime > k.GTH_Arm2HatchTimer){
                gatherState = FourBarState.WAIT;
                inBallPosition = false;
                inHatchPosition = true;
            }
            break;
            
        }

    }


}