package frc.robot.subsystems;

import frc.robot.io.K;

public class Climber extends Component{
    public Climber() {

    }

    public void run(){
        if(in.climb){
            out.climbMotor(K.CLM_MotorSpeed);
        }
        else if(in.reverseclimb){
            out.climbMotor(-K.CLM_MotorSpeed);
        }
        else {
           out.climbMotor(0); 
        }
    }
}
