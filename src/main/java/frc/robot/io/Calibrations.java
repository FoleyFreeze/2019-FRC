package frc.robot.io;

public class Calibrations{
    
    public enum RobotType {
        COMPETITION, PRACTICE, SWERVE_BOT
    }
    public static RobotType BOT_Version = RobotType.PRACTICE;

    public double   DRV_CountsPerDegree;
    public final double[] DRV_WheelLocX = {-12.375,12.375,-12.375,12.375};
    public final double[] DRV_WheelLocY = {10.625,10.625,-10.625,-10.625};
    public double   DRV_RotCentX;
    public double   DRV_RotCentY;
    public double   DRV_SwerveAngRate;
    public double   DRV_SwerveAngKP;
    public double   DRV_SwerveMaxAnglePwr; 
    public double   DRV_SwerveDrivePwrScale;
    public double   DRV_SwerveStrKP;
    public double   DRV_WaitForParkTime; 
    public double SEN_AbsAngleFL;
    public double SEN_AbsAngleFR; 
    public double SEN_AbsAngleRL;
    public double SEN_AbsAngleRR;
    public double GTH_IntakeSpeed;
    public double GTH_ShootSpeedFast;
    public double GTH_ShootSpeedSlow;
    public double ELE_MotorPwr;
    public final double[] ELE_PositionArray = {
        //FLOOR, LOADING_STATION, ROCKET_1_CARGO, ROCKET_1_HATCH, ROCKET_2_CARGO, ROCKET_2_HATCH, ROCKET_3_CARGO, ROCKET_3_HATCH, SHIP_CARGO, SHIP_HATCH 
            0,               0,               0,              0,              0,              0,             0,               0,          0,          0
    };
    public double ELE_PositionKP;
    public double CLM_MotorSpeed;
    public double IN_xDeadband;
    public double IN_yDeadband;
    public double IN_rotDeadband;
    public double IN_xyDeadband;
    public double IN_DodgingMin;
    public int IN_resetGyro;
    public int IN_compassDrive;
    public int IN_fieldOriented;
    public int IN_flipOrientation;
    public int IN_pitMode;
    public int IN_diskGather;
    public int IN_xDriveAxis;
    public int IN_yDriveAxis;
    public int IN_rotDriveAxis;
    public int IN_dodgingL;
    public int IN_dodgingR;
    public int MIL_livepi;
    public int MIL_trackpi;
    public double OUT_PitModeLimit;
    public boolean OUT_DriveBrakeMode;


    
    public Calibrations(){

        DRV_CountsPerDegree = 4161.0/3600.0;//4161 in 10 rotations
        DRV_RotCentX = 0.0;
        DRV_RotCentY = 0.0;
        DRV_SwerveAngRate = 0.05;
        DRV_SwerveAngKP = -0.003;
        DRV_SwerveMaxAnglePwr = 0.5;
        DRV_SwerveDrivePwrScale = 0.5;
        DRV_SwerveStrKP = -0.0;//rotation power per degree
        DRV_WaitForParkTime = 0.5;//seconds

        GTH_IntakeSpeed = 0;
        GTH_ShootSpeedFast = 0;
        GTH_ShootSpeedSlow = 0;

        ELE_MotorPwr = 1; // set to actual value later
        ELE_PositionKP = 0;

        CLM_MotorSpeed = 0;

        IN_xDeadband = 0.05;
        IN_yDeadband = 0.05;
        IN_rotDeadband = 0.2;
        IN_xyDeadband = 0.2;
        IN_DodgingMin = 0.2;
        //Buttons
        IN_resetGyro = 1;
        IN_compassDrive = 5;//for future, unknown button values = -0
        IN_fieldOriented = 6;
        IN_flipOrientation = -0;
        IN_pitMode = -0;
        IN_diskGather = -0;
        //Axes
        IN_xDriveAxis = 0;
        IN_yDriveAxis = 1;
        IN_rotDriveAxis = 4;
        IN_dodgingL = 2;
        IN_dodgingR = 3;
        //Mil
        MIL_livepi = 0;
        MIL_trackpi = 0;


        OUT_PitModeLimit = 0.25;
        OUT_DriveBrakeMode = true;
    }   
}