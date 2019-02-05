package frc.robot.io;

public class Calibrations{
    
    public enum RobotType {
        COMPETITION, PRACTICE, SWERVE_BOT
    }
    public static final RobotType BOT_Version = RobotType.PRACTICE;

    public static  double   DRV_CountsPerDegree;
    public static final double[] DRV_WheelLocX = {-12.375,12.375,-12.375,12.375};
    public static final double[] DRV_WheelLocY = {10.625,10.625,-10.625,-10.625};
    public static  double   DRV_RotCentX;
    public static  double   DRV_RotCentY;
    public static  double   DRV_SwerveAngRate;
    public static  double   DRV_SwerveAngKP;
    public static  double   DRV_SwerveMaxAnglePwr; 
    public static  double   DRV_SwerveDrivePwrScale;
    public static  double   DRV_SwerveStrKP;
    public static  double   DRV_WaitForParkTime; 

    //swerve bot values
    //public static final double SEN_AbsAngleFL = 109.424; //109.424 for swereve bot 
    //public static final double SEN_AbsAngleFR = 343.916;
    //public static final double SEN_AbsAngleRL = 203.818;
    //public static final double SEN_AbsAngleRR = 167.246;

    public static  double SEN_AbsAngleFL;
    public static  double SEN_AbsAngleFR; 
    public static  double SEN_AbsAngleRL;
    public static  double SEN_AbsAngleRR;

    public static  double GTH_IntakeSpeed;
    public static  double GTH_ShootSpeedFast;
    public static  double GTH_ShootSpeedSlow;


    public static  double ELE_MotorPwr;


    public static  double CLM_MotorSpeed;


    public static  double IN_xDeadband;
    public static  double IN_yDeadband;
    public static  double IN_rotDeadband;
    public static  double IN_xyDeadband;
    //Buttons
    public static  int IN_resetGyro;
    public static  int IN_compassDrive;
    public static  int IN_fieldOriented;
    public static  int IN_pitMode;
    //Axes
    public static  int IN_xDriveAxis;
    public static  int IN_yDriveAxis;
    public static  int IN_rotDriveAxis;

    public static  double OUT_PitModeLimit;

    
    public Calibrations(){

        DRV_CountsPerDegree = 4161.0/3600.0;//4161 in 10 rotations
        DRV_RotCentX = 0.0;
        DRV_RotCentY = 0.0;
        DRV_SwerveAngRate = 0.05;
        DRV_SwerveAngKP = -0.003;
        DRV_SwerveMaxAnglePwr = 0.5;
        DRV_SwerveDrivePwrScale = 0.5;
        DRV_SwerveStrKP = -0.1;//rotation power per degree
        DRV_WaitForParkTime = 0.5;//seconds
        SEN_AbsAngleFL = 77.5;
        SEN_AbsAngleFR = 344.7;
        SEN_AbsAngleRL = 232.3;
        SEN_AbsAngleRR = 137;
        GTH_IntakeSpeed = 0;
        GTH_ShootSpeedFast = 0;
        GTH_ShootSpeedSlow = 0;
        ELE_MotorPwr = 1; // set to actual value later
        CLM_MotorSpeed = 0;
        IN_xDeadband = 0.05;
        IN_yDeadband = 0.05;
        IN_rotDeadband = 0.2;
        IN_xyDeadband = 0.2;
        IN_resetGyro = 1;
        IN_compassDrive = 5;//for future, unknown button values = -0
        IN_fieldOriented = 6;
        IN_pitMode = -0;
        IN_xDriveAxis = 0;
        IN_yDriveAxis = 1;
        IN_rotDriveAxis = 4;
        OUT_PitModeLimit = 0.25;
    }   
}