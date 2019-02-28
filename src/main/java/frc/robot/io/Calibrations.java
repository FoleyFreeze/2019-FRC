package frc.robot.io;

public class Calibrations{
    
    public enum RobotType {
        COMPETITION, PRACTICE, SWERVE_BOT
    }
    public static RobotType BOT_Version = RobotType.PRACTICE;

    public double NAVX_Offset;

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
    public double   DRV_SwerveStrKD;
    public double   DRV_WaitForParkTime; 
    public double   DRV_InchesPRev;
    public double DRV_toTargetAngleKP;
    public double DRV_ofTargetAngleKP;
    public double DRV_targetDistanceKP;
    public double SEN_AbsAngleFL;
    public double SEN_AbsAngleFR; 
    public double SEN_AbsAngleRL;
    public double SEN_AbsAngleRR;
    public double GTH_IntakeSpeed;
    public double GTH_ShootSpeedFast;
    public double GTH_ShootSpeedSlow;
    public double GTH_ArmInPwr;
    public double GTH_ArmOutPwr;
    public double GTH_StartUpTime;
    public double GTH_ArmInCurrent;
    public double GTH_ArmOutCurrent;
    public double GTH_FailSafeTimer;
    public double ELE_MotorPwr;
    public final double[] ELE_PositionArray = {
        //FLOOR, LOADING_STATION, ROCKET_1_CARGO, ROCKET_1_HATCH, ROCKET_2_CARGO, ROCKET_2_HATCH, ROCKET_3_CARGO, ROCKET_3_HATCH, SHIP_CARGO, SHIP_HATCH 
            0,               0,               0,              0.5,              0,              0,             0,               78,          0,          0
    };
    public double ELE_PositionKP;
    public double ELE_InchesPRev;
    public double ELE_PIDLimitUp;
    public double ELE_PIDLimitDown;

    public double CLM_MotorSpeedUp;
    public double CLM_MotorSpeedDn;
    
    public double IN_DodgingMin;
    public int IN_DixonSize;
    public int IN_dodgingL;
    public int IN_dodgingR;

    // elevator positions 
    public int IN_rocketL1;
    public int IN_rocketL2;
    public int IN_rocketL3;
    public int IN_rocketSideLeft;
    public int IN_rocketSideRight;


    public int MIL_livepi;
    public int MIL_trackpi;
    public double MIL_CurrResetTime;
    public double MIL_CurrFilt;
    public boolean MIL_CLDisabled;
    public double OUT_PitModeLimit;
    public boolean OUT_DriveBrakeMode;

    // disable cals 
    public boolean ELE_disable;
    public boolean CLM_disable;
    public boolean GTH_disableBall;
    public boolean DRV_disable;
    public boolean GTH_disableDisk;
    public boolean AD_Disabled;
    public boolean MIL_Disabled;

    // Vision cals
    public double VIS_ExpireTime;

    
    public Calibrations(){

        //Disables
        ELE_disable = false;
        CLM_disable = true;
        GTH_disableBall = false;
        GTH_disableDisk = false;
        DRV_disable = false;
        AD_Disabled = true;
        MIL_Disabled = true;
        MIL_CLDisabled = true;

        NAVX_Offset = 0;

        DRV_CountsPerDegree = 4161.0/3600.0;//4161 in 10 rotations
        DRV_RotCentX = 0.0;
        DRV_RotCentY = 0.0;
        DRV_SwerveAngRate = 0.05;
        DRV_SwerveAngKP = -0.003;
        DRV_SwerveMaxAnglePwr = 0.5;
        DRV_SwerveDrivePwrScale = 0.25;
        DRV_SwerveStrKP = -0.2;//rotation power per degree
        DRV_SwerveStrKD = 0;
        DRV_WaitForParkTime = 5;//seconds
        DRV_InchesPRev = 10.0/24.0*0.5*3.0*Math.PI;
        DRV_toTargetAngleKP = 0.05;
        DRV_ofTargetAngleKP = -0; 
        DRV_targetDistanceKP = -0.2;

        GTH_IntakeSpeed = 0.5;
        GTH_ShootSpeedFast = 0.4;
        GTH_ShootSpeedSlow = 0.4;
        GTH_ArmInPwr = 0;
        GTH_ArmOutPwr = 0;
        GTH_StartUpTime = 0;
        GTH_ArmInCurrent = 0;
        GTH_ArmOutCurrent = 0;
        GTH_FailSafeTimer = 0;

        ELE_MotorPwr = 0.1; // set to actual value later
        ELE_PositionKP = 0.1;
        ELE_InchesPRev = 13.0/50.0*20.0/50.0*2.0*Math.PI*75.0/70.0;
        ELE_PIDLimitUp = 0.75;
        ELE_PIDLimitDown = 0.5;

        CLM_MotorSpeedUp = 0.3;
        CLM_MotorSpeedDn = 0.6;
        
        IN_DodgingMin = 0.2;
        IN_DixonSize = 6;
        
        //Buttons
        IN_rocketL1 = -0;
        IN_rocketL2 = -0;
        IN_rocketL3 = -0;
        IN_rocketSideLeft = -0;
        IN_rocketSideRight = -0;
        //Axes
        IN_dodgingL = 2;
        IN_dodgingR = 3;
        //Mil
        MIL_livepi = 0;
        MIL_trackpi = 0;
        MIL_CurrFilt = 0.2; //time constant of 1/val
        MIL_CurrResetTime = 30;


        OUT_PitModeLimit = 0.25;
        OUT_DriveBrakeMode = true;   
        
        VIS_ExpireTime = 0.5;
    }   
}