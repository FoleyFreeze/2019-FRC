package frc.robot.io;

public class K{
    public static final double   DRV_CountsPerDegree = 4161.0/3600.0;//4161 in 10 rotations
    public static final double[] DRV_WheelLocX = {1,2,3,4};
    public static final double[] DRV_WheelLocY = {1,2,3,4};
    public static final double   DRV_RotCentX = 0.0;
    public static final double   DRV_RotCentY = 0.0;
    public static final double   DRV_SwerveAngRate = 0.05;
    public static final double   DRV_SwerveAngKP = -0.03;
    public static final double   DRV_SwerveMaxAnglePwr = 0.5; 

    public static final double SEN_AbsAngleFL = 231.504;
    public static final double SEN_AbsAngleFR = 343.916;
    public static final double SEN_AbsAngleRL = 203.818;
    public static final double SEN_AbsAngleRR = 162.246;

    public static final int FL_DRIVE_ID = -1;
    public static final int FR_DRIVE_ID = -1;
    public static final int BL_DRIVE_ID = -1;
    public static final int BR_DRIVE_ID = -1;
    public static final int FL_TURN_ID = -1;
    public static final int FR_TURN_ID = -1;
    public static final int BL_TURN_ID = -1;
    public static final int BR_TURN_ID = -1;

    public static final int ELE_MotorID = -1;

    public static final int GTH_MotorID = -1;

    public static final int GTH_ArmMotorID = -1;

    public static final int CLM_MotorID = -1;

    public static final double ELE_MotorPwr = 1; // set to actual value later
        
}