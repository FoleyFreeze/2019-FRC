package frc.robot.io;

public class K{
    public static final double   DRV_CountsPerDegree = 4161.0/3600.0;//4161 in 10 rotations
    public static final double[] DRV_WheelLocX = {1,2,3,4};
    public static final double[] DRV_WheelLocY = {1,2,3,4};
    public static final double   DRV_RotCentX = 0.0;
    public static final double   DRV_RotCentY = 0.0;
    public static final double   DRV_SwerveAngRate = 0.05;
    public static final double   DRV_SwerveAngKP = -0.03;
    public static final double   DRV_SwerveMaxAnglePwr = 0.5; 

    public static final int FL_DRIVE_ID = -1;
    public static final int FR_DRIVE_ID = -1;
    public static final int BL_DRIVE_ID = -1;
    public static final int BR_DRIVE_ID = -1;
    public static final int FL_TURN_ID = -1;
    public static final int FR_TURN_ID = -1;
    public static final int BL_TURN_ID = -1;
    public static final int BR_TURN_ID = -1;

    public static final int ELEVATOR_MOTOR_ID = -1;

    public static final int GATHER_MOTOR_ID = -1;

    public static final int GATHER_ARM_MOTOR_ID = -1;

    public static final int CLIMB_MOTOR_ID = -1;

}