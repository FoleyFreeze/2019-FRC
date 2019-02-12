package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DiskGatherer extends Component{

    private enum FourBarState{
        WAIT, STARTUP, WAIT_FOR_STALL
    };
    private FourBarState gatherState = FourBarState.WAIT;

    public DiskGatherer() {
        
    }

    private double startTime;
    private boolean movingIn;

    public void run() {
        String gatherStater;
        if(k.GTH_disableDisk) return;
       
        //make sure that if we disable in a movement we don't re-enable and immediately start moving
        if(sense.isDisabled) gatherState = FourBarState.WAIT;
       
        out.suction(in.diskGather);
        
        switch(gatherState){
            case WAIT:
            out.setGatherArm(0);

            if(in.fourBarIn){
                movingIn = true;
                startTime = Timer.getFPGATimestamp();
                gatherState = FourBarState.STARTUP;    
            } else if(in.fourBarOut){
                movingIn = false;
                startTime = Timer.getFPGATimestamp();
                gatherState = FourBarState.STARTUP;
            }
            gatherStater = "Not Moving";
            break;

            case STARTUP:
            if(movingIn) out.setGatherArm(k.GTH_ArmInPwr);
            else out.setGatherArm(k.GTH_ArmOutPwr);

            if(Timer.getFPGATimestamp() - startTime > k.GTH_StartUpTime){
                gatherState = FourBarState.WAIT_FOR_STALL;
                startTime = Timer.getFPGATimestamp();
            }
            gatherStater = "Starting";
            break;

            case WAIT_FOR_STALL:
            if(movingIn) out.setGatherArm(k.GTH_ArmInPwr);
            else out.setGatherArm(k.GTH_ArmOutPwr);

            if(movingIn && out.getGatherArmCurrent() > k.GTH_ArmInCurrent){
                gatherState = FourBarState.WAIT;
            }else if(!movingIn && out.getGatherArmCurrent() > k.GTH_ArmOutCurrent){
                gatherState = FourBarState.WAIT;
            }else if(Timer.getFPGATimestamp() - startTime > k.GTH_FailSafeTimer){
                gatherState = FourBarState.WAIT;
            }
            gatherStater = "Moving?? I think??";
            break;
            
        }
        //SmartDashboard.putString("State of Gatherer", gatherStater); Fix Later, Arianna doesn't want to break the code

    }


}