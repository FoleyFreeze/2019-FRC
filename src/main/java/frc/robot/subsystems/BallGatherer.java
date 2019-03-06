package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.ElectroJendz;

public class BallGatherer extends Component{

    public BallGatherer() {
        
    }
    
    public void run() {
        String gatherStatus;
        if(k.GTH_disableBall) return;
        
        // conditions for gathering
        if(in.ballGather){
            out.setGatherMotor(k.GTH_CargoIntakeSpeed, -k.GTH_CargoIntakeSpeed);
            gatherStatus = "Gathering";
        }
        //  releases ball
        else if(in.releaseBall){
            out.setGatherMotor(-k.GTH_CargoShootSpeedFast, k.GTH_CargoShootSpeedSlow);
            gatherStatus = "Releasing";
        }
        if(in.diskGather){
            out.setGatherMotor(-k.GTH_DiskIntakeSpeed, k.GTH_DiskIntakeSpeed);
            gatherStatus = "Gathering";
        }
        //  releases ball
        else if(in.releaseDisk){
            out.setGatherMotor(k.GTH_DiskShootSpeedFast, -k.GTH_DiskShootSpeedSlow);
            gatherStatus = "Releasing";
        }
        else if(sense.hasBall){
            gatherStatus = "Hold";
            out.setGatherMotor(k.GTH_HoldSpeed, -k.GTH_HoldSpeed);
        }
        else if(sense.hasHatch){
            gatherStatus = "Hold";
            out.setGatherMotor(-k.GTH_HoldSpeed, k.GTH_HoldSpeed);
        }
        // stop moving
        else {
            out.setGatherMotor(0,0); 
            gatherStatus = "Stopped";
        }
        SmartDashboard.putString("Gather Status", gatherStatus);
    }

} 