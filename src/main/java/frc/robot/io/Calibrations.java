package frc.robot.io;

public class Calibrations{
    
    public enum RobotType {
        COMPETITION, PRACTICE, SWERVE_BOT
    }
    public static RobotType BOT_Version = RobotType.PRACTICE;//doesnt mean anything


    // disable cals
    public boolean AD_Disabled = false;
    public boolean CAM_Disabled = false;
    public boolean CLM_disable = false;
    public boolean DRV_Disable = false;
    public boolean ELE_Disable = false;
    public boolean GTH_DisableGather = false;
    public boolean SCR_ScorpioSelected = false; //enable scorpio instead of normal gatherer
    public boolean MIL_Disabled = false;

    // drive cals
    
    public double DRV_TurnGearRatio = 20/60.0 * 20/60.0 * 38/64.0;
    public boolean DRV_PIDOnSpark = true;
    public double DRV_TurnSparkKP = 0.25;
    public double DRV_TurnSparkKI = 0;
    public double DRV_TurnSparkKD = 0.2;
    public double DRV_TurnSparkKDFilt = 0.5;
    public double DRV_TurnSparkKF = 0;
    public double   DRV_CountsPerDegree = 4161.0/3600.0;//4161 in 10 rotations
    public double DRV_WheelDiameter = 3; //2.9 prac
    public double   DRV_InchesPRev = 10.0/24.0*0.5*DRV_WheelDiameter*Math.PI;
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
    public double   DRV_CamDriveMaxPwr_X = 0.25;
    public double   DRV_CamDriveMaxPwr_Y = 0.25;//.2
    public double   DRV_CamDriveMinPwr_X = 0.02; //was .03 MrC
    public double   DRV_CamDriveMinPwr_Y = 0.00; //was .03 MrC
    public double   DRV_TargetDistanceKP = -0.04;//-0.011 //camera drive kp based on dist
    public double   DRV_TargetDistanceKD = 0.01;
    public double   DRV_CamTargetYoffset = 0;//-2; //Drive 2 inches CLOSER to all targets
    public double   DRV_CamDistShootX = 2;
    public double   DRV_CamDistShootY = 2 - DRV_CamTargetYoffset;
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
    public double AD_MaxPower = 0.7;
    public double AD_AutoDriveKP = -0.01; //power per inch
    public double AD_AutoDriveKD = 0.0025;
    public double AD_RobotWidth = 39; //with bumpers
    public double AD_RobotHeight = 35;
    public double AD_HabY = 48;
    public double AD_HabEdgeX = 64;//real measurement is 64in
    public double AD_MidEdgeX = 24;
    public double AD_LoadingStationX = 135;
    public double AD_AccelLim = 0.4; //x% per second accel
    public double AD_MaxPowerHab = 0.05; //max power on the hab polygons (0 and 18)
    public double AD_BlendDist = 20;//in inches begin blending next target point

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

    public double GTH_CurrLimit = 14.0;//14
    public double GTH_CargoIntakeSpeed = 0.5;
    public double GTH_CargoShootSpeedFast = 0.2;//0.16;//0.13;//0.27;// 0.4; //.2 
    public double GTH_CargoShootSpeedSlow = 0.2;//0.16;//0.13;//0.27;//0.4; //.2
    public double GTH_HatchIntakeSpeed = 0.5;
    public double GTH_HatchShootSpeedFast = 0.4;
    public double GTH_HatchShootSpeedSlow = 0.4;
    public double GTH_CargoHoldSpeed = 0.1; //0.2;
    public double GTH_HatchHoldSpeed = 0.05;
    public double GTH_ReleaseTime = 0.2; //time to spend auto shooting/gathering
    
    public double GTH_CargoGatherTime = 0;
    public double GTH_CargoShootTime = 0.5;
    public double GTH_HatchGatherTime = 0.2;
    public double GTH_HatchShootTime = 0.25;

    //SCORPIO cals
    public double SCR_CargoGatherPwr = -0.5;
    public double SCR_CargoShootPwrRocket = 1;
    public double SCR_CargoShootPwrCShip = 0.2;
    public double SCR_HatchGatherPwr = -0.5;
    public double SCR_HatchShootPwr = 1;
    public double SCR_HatchHoldPwr = -0.10;//-.05;
    public double SCR_CargoHoldPwr = -0.25;

    public double SCR_CargoGatherTime = 0.5;
    public double SCR_CargoShootTime = 0.5;
    public double SCR_HatchGatherTime = 0.5;
    public double SCR_HatchShootTime = 0.25;
    public double SCR_RetractTime = 0.3;
    public double SCR_FullExtendTime = 0.4;
    public double SCR_HalfExtendTime = 0.2;
    public double SCR_ShortExtendTime = 0.02;

    public double SCR_ArmPositionKP = 0.002;//0.002;
    public double SCR_ArmPositionKD = 0;
    public double SCR_ArmIdleHoldPower = -0.15;//-0.15;
    public double SCR_ArmInPower = -0.7;
    public double SCR_ArmOutPower = .7;  //0.9;//was .7
    public double SCR_ArmPowerLimit = 0.7;
    public double SCR_ArmInPowerLimit = 0.4;

    public double SCR_ArmOutCurrentLimit = 25;
    public double SCR_ArmInCurrentLimit = 25;
    public double SCR_WheelCurrentLimit = 15;

    public double SCR_InPosition =  -150;//0;
    public double SCR_FullOutPosition = 1500;
    public double SCR_PartOutPosition = 500;   
    public double SCR_AllowFloorLimit = 350;
    

    // elevator cals
    public double ELE_MotorPwr = 0.1; // set to actual value later
    public double ELE_InchesPRev = 13.0/50.0*20.0/50.0*2.0*Math.PI*75.0/70.0;
    public double ELE_PIDLimitUp = 0.9;
    public double ELE_PIDLimitDown = 0.5;//0.7
    public double ELE_PIDLimitDownLow = 0.1;
    public double ELE_LowLimitPosition = 10;
    public double ELE_PositionKP = 0.1;
    public double ELE_StageHeight = 14;
    public double ELE_PositionOffset = 0.5; //add this number to all the positions
    public final double[] ELE_PositionArray = {
        //FLOOR, LOADING_STATION, ROCKET_1_CARGO, ROCKET_1_HATCH, ROCKET_2_CARGO, ROCKET_2_HATCH, ROCKET_3_CARGO, ROCKET_3_HATCH, SHIP_CARGO, SHIP_HATCH 
            -1,        11.5,           20,              11,              50,              40,           76.5,            68,            34,          11
    };
    //in case scorpio needs different elevator positions
    public final double[] ELE_PositionScorpioArray = {
        //FLOOR, LOADING_STATION, ROCKET_1_CARGO, ROCKET_1_HATCH, ROCKET_2_CARGO, ROCKET_2_HATCH, ROCKET_3_CARGO, ROCKET_3_HATCH, SHIP_CARGO, SHIP_HATCH 
            -0.5,        2,           23,              8,              52,              37,           76.5,            65,            40,          8
    };
    public double ELE_ScorpioFloor = 3.5;//floor when scorpio is retracted


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
    public double CAM_ShootDist = 2;//inches
    public boolean CAM_DebugCargo = false;
    public boolean CAM_DebugTargetHigh = false;
    public boolean CAM_DebugTargetLow = false;
    public boolean CAM_AutoShootDisabled = false; //disabled until driver is ok with this feature
    //public double CAM_AutoShootCargoDist = 2;//inches
    //public double CAM_AutoShootHatchDist = 18;//2;
    //public double CAM_AutoGatherHatchDist = 18;//18

    public double CAM_SCR_MainTargetOffset = 0; //subtract this offset from all targets HIGHER = FARTHER
    public double CAM_SCR_Stage1Offset = 2; //extra offset when the angle isn't aligned yet
    public double CAM_SCR_Cargo_CS_Offset = 2;//-2;//scorpio cargo in cargo ship offset 
    public double CAM_SCR_Cargo_RKT_Offset = 2;//-2;//scorpio cargo in rocket offset
    public double CAM_SCR_Hatch_CS_Offset = 18;//scorpio hatch in cargo ship offset
    public double CAM_SCR_Hatch_RKT_Offset = 18;//scorpio hatch in rocket offset
    public double CAM_SCR_Hatch_LS_Offset = 16;//scorpio hatch in loading station offset
    public double CAM_SCR_Cargo_CS_XErrLimit = 5;//scorpio angle limit for cargo on cargo ship
    public double CAM_SCR_Cargo_RKT_XErrLimit = 5;//arm gather angle limit for cargo in rocket
    public double CAM_SCR_XErrLimit = 1.5;//scorpio angle limit for everything else
    public double CAM_SCR_AllowShootXErr = 1.5; //autoShoot true when within this angle
    public double CAM_SCR_AllowShootCargoCSXErr = 10;//overrides angle when cargo going to cargo ship
    public double CAM_SCR_AllowShootCargoRKTXErr = 10;//overrides angle when cargo going to cargo ship

    public double CAM_GTH_MainTargetOffset = 0; //subtract this offset from all targets
    public double CAM_GTH_Stage1Offset = 6;//8 //extra offset when the angle isn't aligned yet
    public double CAM_GTH_Cargo_CS_Offset = 2;//arm gather cargo in cargo ship offset 
    public double CAM_GTH_Cargo_RKT_Offset = -2;//arm gather cargo in rocket offset
    public double CAM_GTH_Hatch_CS_Offset = 0;//arm gather hatch in cargo ship offset
    public double CAM_GTH_Hatch_RKT_Offset = 2;//arm gather hatch in rocket offset
    public double CAM_GTH_Hatch_LS_Offset = -1;//arm gather hatch in loading station offset
    public double CAM_GTH_Cargo_CS_XErrLimit = 15;//arm gather angle limit for cargo on cargo ship
    public double CAM_GTH_Cargo_RKT_XErrLimit = 2;//arm gather angle limit for cargo in rocket
    public double CAM_GTH_XErrLimit = 1;//1.5;//arm gather angle limit for everything else
    public double CAM_GTH_AllowShootXErr = 0.5;//1.5; //autoShoot true when within this angle
    public double CAM_GTH_AllowShootCargoCSXErr = 5;//overrides angle when cargo going to cargo ship
    public double CAM_GTH_AllowShootCargoRKTXErr = 2;//arm gather angle limit for cargo in rocket

    public Calibrations(){
        
    }   
}