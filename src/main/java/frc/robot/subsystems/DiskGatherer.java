package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DiskGatherer extends Component{

    private enum FourBarState{
        WAIT, STARTUP, WAIT_FOR_STALL
    }
    private FourBarState gatherState = FourBarState.WAIT;

    public DiskGatherer() {
        
    }

    private double startTime;
    private boolean movingToBall;

    public void run() {
        if(k.GTH_disableDisk) return;
       
        //make sure that if we disable in a movement we don't re-enable and immediately start moving
        if(sense.isDisabled) gatherState = FourBarState.WAIT;
       
        //out.suction(in.diskGather);
        
        SmartDashboard.putString("GatherState", gatherState.name());

        switch(gatherState){
            case WAIT:
            out.setGatherArm(0);

            if(in.ballNotHatch && !sense.hasHatch){
                movingToBall = true;
                startTime = Timer.getFPGATimestamp();
                gatherState = FourBarState.STARTUP;    
            } else if(!in.ballNotHatch && !sense.hasBall){
                movingToBall = false;
                startTime = Timer.getFPGATimestamp();
                gatherState = FourBarState.STARTUP;
            }
            break;

            case STARTUP:
            if(movingToBall) out.setGatherArm(k.GTH_ArmInPwr);
            else out.setGatherArm(k.GTH_ArmOutPwr);

            if(Timer.getFPGATimestamp() - startTime > k.GTH_StartUpTime){
                gatherState = FourBarState.WAIT_FOR_STALL;
                startTime = Timer.getFPGATimestamp();
            }
            break;

            case WAIT_FOR_STALL:
            if(movingToBall) out.setGatherArm(k.GTH_ArmInPwr);
            else out.setGatherArm(k.GTH_ArmOutPwr);

            if(movingToBall && out.getGatherArmCurrent() > k.GTH_ArmInCurrent){
                gatherState = FourBarState.WAIT;
            }else if(!movingToBall && out.getGatherArmCurrent() > k.GTH_ArmOutCurrent){
                gatherState = FourBarState.WAIT;
            }else if(Timer.getFPGATimestamp() - startTime > k.GTH_FailSafeTimer){
                gatherState = FourBarState.WAIT;
            }
            break;
            
        }

    }


}