package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.ElectroJendz;

public class BallGatherer extends Component{

    public BallGatherer() {
        
    }
    
    boolean currStop = false;
    public void run() {
        String gatherStatus;
        if(k.GTH_disableBall) return;
        
        if(in.ballGather){
            if(currStop){
                out.setGatherMotor(0.05, -0.05);
            } else {
                out.setGatherMotor(k.GTH_IntakeSpeed, -k.GTH_IntakeSpeed);
            }
            if(sense.pdp.getCurrent(ElectroJendz.GTH_MotorL_ID) > 14){
                currStop = true;
            }
            gatherStatus = "Gathering";
        }
        else if(in.releaseBall){
            currStop = false;
            out.setGatherMotor(-k.GTH_ShootSpeedFast, k.GTH_ShootSpeedSlow);
            gatherStatus = "Releasing";
        }
        else {
            currStop = false;
            out.setGatherMotor(0,0); 
            gatherStatus = "Not Moving the Wheels";
        }
        SmartDashboard.putString("Cargo Gather Status", gatherStatus);
    }

} 