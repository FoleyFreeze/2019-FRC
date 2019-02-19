package frc.robot.io;

public class K_Competition_Bot extends Calibrations{   
  
    public K_Competition_Bot(){
        super();
       
        SEN_AbsAngleFL = 77.5;
        SEN_AbsAngleFR = 278.7;
        SEN_AbsAngleRL = 232.3;
        SEN_AbsAngleRR = 137;

        DRV_SwerveAngKP = -0.01;
        DRV_SwerveStrKP = -0.02;
    }   
}