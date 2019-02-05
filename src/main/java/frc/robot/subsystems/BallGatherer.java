package frc.robot.subsystems;

import frc.robot.io.Calibrations;

public class BallGatherer extends Component{

    public BallGatherer() {

    }
    
    public void run() {
        if(in.ballGather){
            out.setGatherMotor(Calibrations.GTH_IntakeSpeed, Calibrations.GTH_IntakeSpeed);
        }
        else if(in.releaseBall){
            out.setGatherMotor(Calibrations.GTH_ShootSpeedFast, Calibrations.GTH_ShootSpeedSlow);
        }
        else {
            out.setGatherMotor(0,0); 
        }
    }

} 