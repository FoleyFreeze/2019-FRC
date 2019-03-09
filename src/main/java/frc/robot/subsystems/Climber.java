package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Component{
    public Climber() {

    }

    public void run(){
        String climb;
        if(k.CLM_disable) return;
        
        double pwr = (in.gamePad.getRawAxis(6) + 1) /2;
        SmartDashboard.putNumber("Climb Power", pwr);

        if(in.climb){
            if(sense.climberEncoder > k.CLM_EncoderLimit){
                out.climbMotor(0);
            } else {
                if(sense.climberEncoder < k.CLM_Zone_1){
                    out.climbMotor(k.CLM_Zone_Power_1);
                } 
                else if(sense.climberEncoder < k.CLM_Zone_2){
                    out.climbMotor(k.CLM_Zone_Power_2);
                }
                else if(sense.climberEncoder < k.CLM_Zone_3){
                    out.climbMotor(k.CLM_Zone_Power_3);
                }
                else{
                    out.climbMotor(0);
                }


                //out.climbMotor(pwr);
            }
            
            climb = "Climbing";
        }
        else if(in.reverseClimb){
            out.climbMotor(-pwr);
            climb = "Climbing in Reverse";
        }
        else {
           out.climbMotor(0); 
           climb = "Nope";
        }
        SmartDashboard.putString("Climb Status", climb);
    }
}
