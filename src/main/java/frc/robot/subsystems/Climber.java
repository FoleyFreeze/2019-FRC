package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Component{
    public Climber() {

    }

    public int stage;

    public void run(){
        String climb;
        if(k.CLM_disable) return;

        if(in.climb){
            if(sense.climberEncoder > k.CLM_EncoderLimit){
                out.climbMotor(0);
                stage = 4;
            } else {

                if(sense.climberEncoder < k.CLM_Zone_1){
                    out.climbMotor(k.CLM_Zone_Power_1);
                    //if(sense.climberEncoder < k.CLM_Zone_Half){
                    //    stage = 0;
                    //} else {
                        stage = 1;
                    //}
                } 
                else if(sense.climberEncoder < k.CLM_Zone_2){
                    out.climbMotor(k.CLM_Zone_Power_2);
                    stage = 2;
                }
                else if(sense.climberEncoder < k.CLM_Zone_3){
                    out.climbMotor(k.CLM_Zone_Power_3);
                    stage = 3;
                }
                else{
                    stage = 0;
                    out.climbMotor(0);
                }


                //out.climbMotor(pwr);
            }
            
            climb = "Climbing";
        }
        else if(in.reverseClimb){
            out.climbMotor(-k.CLM_MotorSpeedDn);
            climb = "Climbing in Reverse";
        }
        else {
           out.climbMotor(0); 
           climb = "Nope";
        }
        //SALwasHere-SmartDashboard.putString("Climb Status", climb);
    }
}
