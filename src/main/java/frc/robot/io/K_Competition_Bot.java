package frc.robot.io;

public class K_Competition_Bot extends Calibrations{   
  
    public K_Competition_Bot(){
        super();
        
        if(Calibrations.BOT_Version == RobotType.PRACTICE){
            SEN_AbsAngleFL = 77.5;
            SEN_AbsAngleFR = 278.7;
            SEN_AbsAngleRL = 232.3;
            SEN_AbsAngleRR = 137;
        } else {
            SEN_AbsAngleFL = -24.4;
            SEN_AbsAngleFR = -36.3;
            SEN_AbsAngleRL = 82.6;
            SEN_AbsAngleRR = -34.3;
        }


        DRV_SwerveAngKP = -0.01;
        DRV_SwerveStrKP = -0.02;
    }   
}