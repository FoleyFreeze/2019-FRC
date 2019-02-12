package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Component{
    public Climber() {

    }

    public void run(){
        String climb;
        if(k.CLM_disable) return;
        
        if(in.climb){
            out.climbMotor(k.CLM_MotorSpeed);
            climb = "Climbing";
        }
        else if(in.reverseclimb){
            out.climbMotor(-k.CLM_MotorSpeed);
            climb = "Climbing in Reverse";
        }
        else {
           out.climbMotor(0); 
           climb = "Nope";
        }
        SmartDashboard.putString("Climb Status", climb);
    }
}
