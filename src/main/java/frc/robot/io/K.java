package frc.robot.io;

public class K{
    
    public enum RobotType {
        COMPETITION, PRACTICE, SWERVE_BOT
    }
    public static final RobotType BOT_Version = RobotType.COMPETITION;

    public static final double   DRV_CountsPerDegree = 4161.0/3600.0;//4161 in 10 rotations
    public static final double[] DRV_WheelLocX = {1,2,3,4};
    public static final double[] DRV_WheelLocY = {1,2,3,4};
    public static final double   DRV_RotCentX = 0.0;
    public static final double   DRV_RotCentY = 0.0;
    public static final double   DRV_SwerveAngRate = 0.05;
    public static final double   DRV_SwerveAngKP = -0.03;
    public static final double   DRV_SwerveMaxAnglePwr = 0.5; 
    public static final double   DRV_SwerveStrKP = 0;//rotation power per degree
    public static final double   DRV_WaitForParkTime = 2; 

    public static final double SEN_AbsAngleFL = 231.504;
    public static final double SEN_AbsAngleFR = 343.916;
    public static final double SEN_AbsAngleRL = 203.818;
    public static final double SEN_AbsAngleRR = 162.246;


    public static final double GTH_IntakeSpeed = 0;
    public static final double GTH_ShootSpeedFast = 0;
    public static final double GTH_ShootSpeedSlow = 0;


    public static final double ELE_MotorPwr = 0.25; // set to actual value later
    public static final double[] ELE_PositionArray = {
        //FLOOR, LOADING_STATION, ROCKET_1_CARGO, ROCKET_1_HATCH, ROCKET_2_CARGO, ROCKET_2_HATCH, ROCKET_3_CARGO, ROCKET_3_HATCH, SHIP_CARGO, SHIP_HATCH 
            0,               0,               0,              0,              0,              0,             0,               0,          0,          0
    };
    public static final double ELE_PositionKP = 0;


    public static final double CLM_MotorSpeed = 0;


    public static final double IN_xDeadband = 0.05;
    public static final double IN_yDeadband = 0.05;
    public static final double IN_rotDeadband = 0.2;
    public static final double IN_xyDeadband = 0.2;
    //Buttons
    public static final int IN_resetGyro = 1;
    public static final int IN_compassDrive = -0;//for future, unknown button values = -0
    public static final int IN_fieldOriented = -0;
    //Axes
    public static final int IN_xDriveAxis = 0;
    public static final int IN_yDriveAxis = 1;
    public static final int IN_rotDriveAxis = 4;
    
}