package frc.robot.io;

public class Calibrations{
    
    public enum RobotType {
        COMPETITION, PRACTICE, SWERVE_BOT
    }
    public static RobotType BOT_Version = RobotType.PRACTICE;

    public double NAVX_Offset;

    // disable cals
    public boolean AD_Disabled = true;
    public boolean CAM_Disabled = false;
    public boolean CLM_disable = true;
    public boolean DRV_disable = false;
    public boolean ELE_disable = false;
    public boolean GTH_disableBall = false;
    public boolean GTH_disableDisk = false;
    public boolean MIL_Disabled = false;

    // drive cals
    public double   DRV_CountsPerDegree = 4161.0/3600.0;//4161 in 10 rotations
    public double   DRV_InchesPRev = 10.0/24.0*0.5*3.0*Math.PI;
    public double   DRV_RotCentX = 0.0;
    public double   DRV_RotCentY = 0.0;
    public double   DRV_SwerveAngRate = 0.05;
    public double   DRV_SwerveAngKP = -0.003;
    public double   DRV_SwerveMaxAnglePwr = 0.5;
    public double   DRV_SwerveDrivePwrScale = 0.25;
    public double   DRV_SwerveStrKP = -0.2;//rotation power per degree
    public double   DRV_SwerveStrKD = 0;
    public double   DRV_toTargetAngleKP  = 0.05;
    public double   DRV_ofTargetAngleKP = -0; 
    public double   DRV_targetDistanceKP  = -0.2;
    public double   DRV_WaitForParkTime  = 5;//seconds
    public final double[] DRV_WheelLocX = {-12.375,12.375,-12.375,12.375};
    public final double[] DRV_WheelLocY = {10.625,10.625,-10.625,-10.625};

    //sensor cals
    public double SEN_AbsAngleFL;
    public double SEN_AbsAngleFR; 
    public double SEN_AbsAngleRL;
    public double SEN_AbsAngleRR;

    // gatherer cals
    public double GTH_ArmInCurrent = 0;
    public double GTH_ArmOutCurrent = 0;
    public double GTH_ArmInPwr = 0;
    public double GTH_ArmOutPwr = 0;
    public double GTH_CurrLimit = 14.0;
    public double GTH_FailSafeTimer = 0;
    public double GTH_IntakeSpeed = 0.5;
    public double GTH_ShootSpeedFast = 0.4;
    public double GTH_ShootSpeedSlow = 0.4;
    public double GTH_StartUpTime = 0;

    // elevator cals
    public double ELE_MotorPwr = 0.1; // set to actual value later
    public double ELE_InchesPRev = 13.0/50.0*20.0/50.0*2.0*Math.PI*75.0/70.0;
    public double ELE_PIDLimitUp = 0.75;
    public double ELE_PIDLimitDown = 0.5;
    public double ELE_PositionKP = 0.1;
    public final double[] ELE_PositionArray = {
        //FLOOR, LOADING_STATION, ROCKET_1_CARGO, ROCKET_1_HATCH, ROCKET_2_CARGO, ROCKET_2_HATCH, ROCKET_3_CARGO, ROCKET_3_HATCH, SHIP_CARGO, SHIP_HATCH 
            0,               0,               0,              0.5,              0,              0,             0,               78,          0,          0
    };

    // climb cals
    public double CLM_MotorSpeedUp = 0.3;
    public double CLM_MotorSpeedDn = 0.6;
    
    // input cals
    public double IN_DodgingMin = 0.2;
    public int IN_DixonSize = 6;
    public int IN_dodgingL = 2;
    public int IN_dodgingR = 3;

    // elevator buttons 
    public int IN_rocketL1 = -0;//new button int in controlboard branch(ControlBoard file), yes it exists on GitHub
    public int IN_rocketL2 = -0;//new button int in controlboard branch(ControlBoard file), yes it exists on GitHub
    public int IN_rocketL3 = -0;//new button int in controlboard branch(ControlBoard file), yes it exists on GitHub
    public int IN_rocketSideLeft = -0;
    public int IN_rocketSideRight = -0;

    //mils
    public int MIL_livepi = 0;
    public int MIL_trackpi = 0;
    public double MIL_CurrResetTime = 30;
    public double MIL_CurrFilt = 0.2; //time constant of 1/val
    public boolean MIL_CLDisabled = true;

    // output cals
    public double OUT_PitModeLimit = 0.25;
    public boolean OUT_DriveBrakeMode = true;

    // Vision cals
    public double VIS_ExpireTime = 0.5;
    
    public Calibrations(){
        
    }   
}