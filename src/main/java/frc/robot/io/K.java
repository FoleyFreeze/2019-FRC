package frc.robot.io;

public class K{
    
    public enum RobotType {
        COMPETITION, PRACTICE, SWERVE_BOT
    }
    public static final RobotType BOT_Version = RobotType.PRACTICE;

    public static final double   DRV_CountsPerDegree = 4161.0/3600.0;//4161 in 10 rotations
    public static final double[] DRV_WheelLocX = {-12.375,12.375,-12.375,12.375};
    public static final double[] DRV_WheelLocY = {10.625,10.625,-10.625,-10.625};
    public static final double   DRV_RotCentX = 0.0;
    public static final double   DRV_RotCentY = 0.0;
    public static final double   DRV_SwerveAngRate = 0.05;
    public static final double   DRV_SwerveAngKP = -0.003;
    public static final double   DRV_SwerveMaxAnglePwr = 0.5; 
    public static final double   DRV_SwerveDrivePwrScale = 0.5;
    public static final double   DRV_SwerveStrKP = -0.1;//rotation power per degree
    public static final double   DRV_WaitForParkTime = 0.5;//seconds 


    //swerve bot values
    //public static final double SEN_AbsAngleFL = 109.424;
    //public static final double SEN_AbsAngleFR = 343.916;
    //public static final double SEN_AbsAngleRL = 203.818;
    //public static final double SEN_AbsAngleRR = 167.246;

    public static final double SEN_AbsAngleFL = 77.5;
    public static final double SEN_AbsAngleFR = 344.7;
    public static final double SEN_AbsAngleRL = 232.3;
    public static final double SEN_AbsAngleRR = 137;

    public static final double GTH_IntakeSpeed = 0;
    public static final double GTH_ShootSpeedFast = 0;
    public static final double GTH_ShootSpeedSlow = 0;


    public static final double ELE_MotorPwr = 1; // set to actual value later


    public static final double CLM_MotorSpeed = 0;

    //overiding joystick if values are too small
    public static final double IN_xDeadband = 0.05;
    public static final double IN_yDeadband = 0.05;
    public static final double IN_rotDeadband = 0.2;
    public static final double IN_xyDeadband = 0.2;
    public static final double IN_DodgingMin = 0.2;

    //Buttons
    public static final int IN_resetGyro = 1;
    public static final int IN_compassDrive = 5;//for future, unknown button values = -0
    public static final int IN_fieldOriented = 6;
    public static final int IN_pitMode = -0;
    public static final int IN_diskGather = -0;
    //Axes
    public static final int IN_xDriveAxis = 0;
    public static final int IN_yDriveAxis = 1;
    public static final int IN_rotDriveAxis = 4;

    public static final double OUT_PitModeLimit = 0.25;
    public static final int IN_dodgingL = 2;
    public static final int IN_dodgingR = 3;
    
}