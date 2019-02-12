package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallGatherer extends Component{

    public BallGatherer() {
        
    }
    
    public void run() {
        String gatherStatus;
        if(k.GTH_disableBall) return;
        
        if(in.ballGather){
            out.setGatherMotor(k.GTH_IntakeSpeed, k.GTH_IntakeSpeed);
            gatherStatus = "Gathering";
        }
        else if(in.releaseBall){
            out.setGatherMotor(k.GTH_ShootSpeedFast, k.GTH_ShootSpeedSlow);
            gatherStatus = "Releasing";
        }
        else {
            out.setGatherMotor(0,0); 
            gatherStatus = "Not Moving";
        }
        SmartDashboard.putString("Cargo Gather Status", gatherStatus);
    }

} 