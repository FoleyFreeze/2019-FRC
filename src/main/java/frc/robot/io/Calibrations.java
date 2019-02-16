package frc.robot.io;

import frc.robot.io.ButtonMap;

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
    public double   DRV_WaitForParkTime; 
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
            0,               0,               0,              0,              0,              0,             0,               0,          0,          0
    };
    public double ELE_PositionKP;
    public double CLM_MotorSpeed;
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


    public double OUT_PitModeLimit;
    public boolean OUT_DriveBrakeMode;

    // disable cals 
    public boolean ELE_disable;
    public boolean CLM_disable;
    public boolean GTH_disableBall;
    public boolean DRV_disable;
    public boolean GTH_disableDisk;

    
    public Calibrations(){

        //Disables
        ELE_disable = true;
        CLM_disable = true;
        GTH_disableBall = true;
        GTH_disableDisk = true;
        DRV_disable = false;

        NAVX_Offset = 0.0;//was 90.0

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
        GTH_ArmInPwr = 0;
        GTH_ArmOutPwr = 0;
        GTH_StartUpTime = 0;
        GTH_ArmInCurrent = 0;
        GTH_ArmOutCurrent = 0;
        GTH_FailSafeTimer = 0;

        ELE_MotorPwr = 1; // set to actual value later
        ELE_PositionKP = 0;

        CLM_MotorSpeed = 0;
        
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

        OUT_PitModeLimit = 0.25;
        OUT_DriveBrakeMode = true;        
    }   
}