package frc.robot.subsystems;

import frc.robot.io.K;

public class BallGatherer extends Component{

    public BallGatherer() {

    }
    
    public void run() {
        if(in.ballGather){
            out.setGatherMotor(K.GTH_IntakeSpeed, K.GTH_IntakeSpeed);
        }
        else if(in.releaseBall){
            out.setGatherMotor(K.GTH_ShootSpeedFast, K.GTH_ShootSpeedSlow);
        }
        else {
            out.setGatherMotor(0,0); 
        }
    }

} 