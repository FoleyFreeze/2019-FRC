package frc.robot.io;

public class K_Competition_Bot extends Calibrations{   
  
    public K_Competition_Bot(){
        super();
        
        if(Calibrations.BOT_Version == RobotType.PRACTICE){
            SEN_AbsAngleFL = 75.41;
            SEN_AbsAngleFR = -17.44;
            SEN_AbsAngleRL = 50.80;
            SEN_AbsAngleRR = -140.88;

            AD_CargoShip_Xoffset = 6;
        } else {
            SEN_AbsAngleFL = .08789;
            SEN_AbsAngleFR = 21.533;
            SEN_AbsAngleRL = -105.117;
            SEN_AbsAngleRR = -39.814;
        }


        DRV_SwerveAngKP = -0.005;
        DRV_SwerveStrKP = -0.01;
    }   
}