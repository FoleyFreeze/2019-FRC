package frc.robot.io;

public class Calibrations{
    
    public enum RobotType {
        COMPETITION, PRACTICE, SWERVE_BOT
    }
    public static RobotType BOT_Version = RobotType.PRACTICE;


    // disable cals
    public boolean AD_Disabled = false;
    public boolean CAM_Disabled = false;
    public boolean CLM_disable = false;
    public boolean DRV_Disable = false;
    public boolean ELE_Disable = false;
    public boolean GTH_DisableCargo = false;
    public boolean GTH_DisableHatch = false;
    public boolean MIL_Disabled = false;

    // drive cals
    
    public double   DRV_CountsPerDegree = 4161.0/3600.0;//4161 in 10 rotations
    public double   DRV_InchesPRev = 10.0/24.0*0.5*3.0*Math.PI;
    public double   DRV_RotCentX = 0.0;
    public double   DRV_RotCentY = 0.0;
    public double   DRV_SwerveAngKP = -0.002;
    public double   DRV_SwerveAngRate = 0.06; //ratio of turn power to wheel distance from center
    public double   DRV_SwerveDrivePwrScale = 1;//0.85; 
    public double   DRV_SwerveMaxAnglePwr = 0.5; //angle pid power limit
    public double   DRV_MotorTurnRampRate = 0.3; //number of seconds to go from 0 to 1 power
    public double   DRV_MotorDriveRampRate = 0.3;//number of seconds to go from 0 to 1 power
    public double   DRV_SwerveStrKD = 0;
    public double   DRV_SwerveStrKP = -0.0;//rotation power per degree
    public double   DRV_DriveStraightDelay = 0.3; //seconds
    public double   DRV_CamDriveMaxPwr_X = 0.3;
    public double   DRV_CamDriveMaxPwr_Y = 0.3;//.2
    public double   DRV_CamDriveMinPwr_X = 0.10; //was .03 MrC
    public double   DRV_CamDriveMinPwr_Y = 0.08; //was .03 MrC
    public double   DRV_TargetDistanceKP = -0.035;//-0.03;//-0.014;//-0.0145;//-0.016; //camera drive kp based on dist
    public double   DRV_TargetDistanceKD = 0.009;//0.4;//0.15;
    public double   DRV_CamTargetY0 = 0;//-2; //Drive 2 inches CLOSER to all targets
    public double   DRV_CamDistShootX = 2;
    public double   DRV_CamDistShootY = 2 - DRV_CamTargetY0;
    public double   DRV_WaitForParkTime = 1;//seconds //was 2
    public final double[] DRV_WheelLocX = {-12.375,12.375,-12.375,12.375};
    public final double[] DRV_WheelLocY = {10.625,10.625,-10.625,-10.625};
    public final double[] DRV_EleHeightAxis = {0,20,78};
    public final double[] DRV_PowerTable = {1, 1, 0.2};
    public double DRV_AutoRotateKP = 0.01; //power per degres of error
    public double DRV_AutoRotateKD = 0.01; //via late night testing
    public double DRV_AutoRotatePwr = 0.3; //max auto rotate power
    public double DRV_AxisExpo = 1.4; //drive axes are powered by this value
    public boolean DRV_DisableAutoOrient = false; //disable PID to correct robot orientation
    public double DRV_CamCargoThetaKP = 0.01; //power per degree
    public double DRV_CamCargoThetaKD = 0.01;
    public double DRV_CamCargoDistKP = -0.02; //power per inch
    public double DRV_CamCargoPwrLim = 0.5;
	public double DRV_CamHatchDeliverForwardPower = 0.2;

    //auto drive cals
    public double AD_MaxPower = 0.3;
    public double AD_AutoDriveKP = -0.01; //power per inch
    public double AD_RobotWidth = 39; //with bumpers
    public double AD_RobotHeight = 35;
    public double AD_HabY = 48;
    public double AD_HabEdgeX = 64;
    public double AD_MidEdgeX = 24;
    public double AD_LoadingStationX = 135;

    //sensor cals
    public double SEN_AbsAngleFL;
    public double SEN_AbsAngleFR; 

    public double SEN_AbsAngleRL;
    public double SEN_AbsAngleRR;
    public double SEN_NAVX_Offset = 0;
    public double SEN_ElevatorEncOffset = 0; //starting elevator height

    // gatherer cals
    public double GTH_ArmOutCurrent = 7;
    public double GTH_ArmInCurrent = 7;
    public double GTH_Arm2CargoPwr = 1;
    public double GTH_Arm2HatchPwr = 1;
    public double GTH_StartUpTime = 0.2;
    public double GTH_Arm2CargoTimer = 0.5;
    public double GTH_Arm2HatchTimer = 1.75;
    public double GTH_ArmExtraIdlePwr = 0.20; //power applied after closing to prevent backdriving
    public double GTH_StartTimer = 0.75; //seconds after enabled before the gatherer can run

    public double GTH_CurrLimit = 14.0;
    public double GTH_CargoIntakeSpeed = 0.5;
    public double GTH_CargoShootSpeedFast = 0.2;//0.16;//0.13;//0.27;// 0.4; //.2 
    public double GTH_CargoShootSpeedSlow = 0.2;//0.16;//0.13;//0.27;//0.4; //.2
    public double GTH_HatchIntakeSpeed = 0.5;
    public double GTH_HatchShootSpeedFast = 0.4;
    public double GTH_HatchShootSpeedSlow = 0.4;
    public double GTH_CargoHoldSpeed = 0.1; //0.2;
    public double GTH_HatchHoldSpeed = 0.05;
    public double GTH_ReleaseTime = 0.2; //time to spend auto shooting/gathering

    // elevator cals
    public double ELE_MotorPwr = 0.1; // set to actual value later
    public double ELE_InchesPRev = 13.0/50.0*20.0/50.0*2.0*Math.PI*75.0/70.0;
    public double ELE_PIDLimitUp = 0.9;
    public double ELE_PIDLimitDown = 0.7;//0.5
    public double ELE_PIDLimitDownLow = 0.2;
    public double ELE_LowLimitPosition = 10;
    public double ELE_PositionKP = 0.1;
    public double ELE_StageHeight = 14;
    public double ELE_PositionOffset = 0.5; //add this number to all the positions
    public final double[] ELE_PositionArray = {
        //FLOOR, LOADING_STATION, ROCKET_1_CARGO, ROCKET_1_HATCH, ROCKET_2_CARGO, ROCKET_2_HATCH, ROCKET_3_CARGO, ROCKET_3_HATCH, SHIP_CARGO, SHIP_HATCH 
            -1,        11,           20,              11,              50,              40,           76.5,            68,            34,          11
    };

    // climb cals
    public double CLM_MotorSpeedUp = 0.15;
    public double CLM_MotorSpeedDn = 0.15;
    public double CLM_EncoderLimit = 65;
    public double CLM_Zone_1 = 20;
    public double CLM_Zone_Half = 10;
    public double CLM_Zone_2 = 60;
    public double CLM_Zone_3 = CLM_EncoderLimit;
    public double CLM_Zone_Power_1 = 0.6; //.6
    public double CLM_Zone_Power_2 = 1;
    public double CLM_Zone_Power_3 = 0.05;
    public double CLM_DrivePower = 0.4; // .3 //how fast to drive at the wall after/during the climb


    
    // input cals
    public double IN_DodgingMin = 0.2;
    public int IN_DixonSize = 6;
    public int IN_dodgingL = 2;
    public int IN_dodgingR = 3;

    //mils
    public int MIL_livepi = 0;
    public int MIL_trackpi = 0;
    public double MIL_CurrResetTime = 5;//seconds to set pit mode when limit exceeded
    public double MIL_CurrFilt = 1; //time constant of 1/val seconds
    public boolean MIL_CLDisabled = false;

    // output cals
    public double OUT_PitModeLimit = 0.2;
    public boolean OUT_DriveBrakeMode = true;

    // Vision cals
    public double CAM_ExpireTime = 0.125; //seconds until image is deleted
    public double CAM_Location_X = 5.125;
    public double CAM_Location_Y = 0;
    public double CAM_ShootDist = 3;//inches
    public boolean CAM_DebugCargo = false;
    public boolean CAM_DebugTargetHigh = false;
    public boolean CAM_DebugTargetLow = false;
    public boolean CAM_AutoShootDisabled = false; //disabled until driver is ok with this feature
    public double CAM_AutoShootCargoDist = 2;//inches
    public double CAM_AutoShootHatchDist = 2;
    public double CAM_AutoGatherHatchDist = 18;
    
    public Calibrations(){
        
    }   
}