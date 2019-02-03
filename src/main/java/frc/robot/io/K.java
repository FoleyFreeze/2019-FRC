package frc.robot.io;

public class K{
    
    public enum RobotType {
        COMPETITION, PRACTICE, SWERVE_BOT
    }
    public static final RobotType BOT_Version = RobotType.SWERVE_BOT;

    public static final double   DRV_CountsPerDegree = 4161.0/3600.0;//4161 in 10 rotations
    public static final double[] DRV_WheelLocX = {-11.625,11.625,-11.625,11.625};
    public static final double[] DRV_WheelLocY = {11,11,-11,-11};
    public static final double   DRV_RotCentX = 0.0;
    public static final double   DRV_RotCentY = 0.0;
    public static final double   DRV_SwerveAngRate = 0.05;
    public static final double   DRV_SwerveAngKP = -0.03;
    public static final double   DRV_SwerveMaxAnglePwr = 1.0; 
    public static final double   DRV_SwerveDrivePwrScale = .5;
    public static final double   DRV_SwerveStrKP = 0;//rotation power per degree
    public static final double   DRV_WaitForParkTime = 0.5;//seconds 

    public static final double SEN_AbsAngleFL = 109.424; //109.424 for swereve bot 
    public static final double SEN_AbsAngleFR = 343.916;
    public static final double SEN_AbsAngleRL = 203.818;
    public static final double SEN_AbsAngleRR = 167.246;


    public static final double GTH_IntakeSpeed = 0;
    public static final double GTH_ShootSpeedFast = 0;
    public static final double GTH_ShootSpeedSlow = 0;


    public static final double ELE_MotorPwr = 1; // set to actual value later


    public static final double CLM_MotorSpeed = 0;


    public static final double IN_xDeadband = 0.05;
    public static final double IN_yDeadband = 0.05;
    public static final double IN_rotDeadband = 0.2;
    public static final double IN_xyDeadband = 0.2;
    //Buttons
    public static final int IN_resetGyro = 1;
    public static final int IN_compassDrive = 5;//for future, unknown button values = -0
    public static final int IN_fieldOriented = 6;
    //Axes
    public static final int IN_xDriveAxis = 0;
    public static final int IN_yDriveAxis = 1;
    public static final int IN_rotDriveAxis = 4;
    
}