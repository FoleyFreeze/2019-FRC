package frc.robot.subsystems;

import frc.robot.io.Calibrations;

public class Climber extends Component{
    public Climber() {

    }

    public void run(){
        if(in.climb){
            out.climbMotor(Calibrations.CLM_MotorSpeed);
        }
        else if(in.reverseclimb){
            out.climbMotor(-Calibrations.CLM_MotorSpeed);
        }
        else {
           out.climbMotor(0); 
        }
    }
}
