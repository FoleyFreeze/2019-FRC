package frc.robot.io;

public class K_Competition_Bot extends Calibrations{   
  
    public K_Competition_Bot(){
        super();
        
        if(Calibrations.BOT_Version == RobotType.PRACTICE){
            SEN_AbsAngleFL = 75.41;
            SEN_AbsAngleFR = -17.44;
            SEN_AbsAngleRL = 50.80;
            SEN_AbsAngleRR = -140.88;
        } else {
            SEN_AbsAngleFL = -24.4;
            SEN_AbsAngleFR = -36.3;
            SEN_AbsAngleRL = 82.6;
            SEN_AbsAngleRR = -34.3;
        }


        DRV_SwerveAngKP = -0.005;
        DRV_SwerveStrKP = -0.01;
    }   
}