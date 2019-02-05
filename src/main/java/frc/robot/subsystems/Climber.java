package frc.robot.subsystems;

public class Climber extends Component{
    public Climber() {

    }

    public void run(){
        if(in.climb){
            out.climbMotor(k.CLM_MotorSpeed);
        }
        else if(in.reverseclimb){
            out.climbMotor(-k.CLM_MotorSpeed);
        }
        else {
           out.climbMotor(0); 
        }
    }
}
