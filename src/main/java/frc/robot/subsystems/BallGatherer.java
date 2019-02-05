package frc.robot.subsystems;

public class BallGatherer extends Component{

    public BallGatherer() {

    }
    
    public void run() {
        if(in.ballGather){
            out.setGatherMotor(k.GTH_IntakeSpeed, k.GTH_IntakeSpeed);
        }
        else if(in.releaseBall){
            out.setGatherMotor(k.GTH_ShootSpeedFast, k.GTH_ShootSpeedSlow);
        }
        else {
            out.setGatherMotor(0,0); 
        }
    }

} 