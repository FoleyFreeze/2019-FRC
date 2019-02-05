package frc.robot.io;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class OutputsCompBot extends Outputs {
        
    private CANSparkMax frontRightMotorDrive;
    private CANSparkMax frontRightMotorTurn;
    private CANSparkMax frontLeftMotorDrive;
    private CANSparkMax frontLeftMotorTurn;
    private CANSparkMax backRightMotorDrive;
    private CANSparkMax backRightMotorTurn;
    private CANSparkMax backLeftMotorDrive;
    private CANSparkMax backLeftMotorTurn;

    private CANSparkMax elevatorMotor;

    private CANSparkMax gatherMotorL;
    private CANSparkMax gatherMotorR;
    private CANSparkMax gatherArmMotor;

    private CANSparkMax climbMotor;
    

    public OutputsCompBot() {
       
        frontLeftMotorDrive = new CANSparkMax(ElectroJendz.FL_DRIVE_ID, MotorType.kBrushless);
        frontLeftMotorTurn = new CANSparkMax(ElectroJendz.FL_TURN_ID, MotorType.kBrushless);
        frontRightMotorDrive = new CANSparkMax(ElectroJendz.FR_DRIVE_ID, MotorType.kBrushless);
        frontRightMotorTurn = new CANSparkMax(ElectroJendz.FR_TURN_ID, MotorType.kBrushless);
        backLeftMotorDrive = new CANSparkMax(ElectroJendz.BL_DRIVE_ID, MotorType.kBrushless);
        backLeftMotorTurn = new CANSparkMax(ElectroJendz.BL_TURN_ID, MotorType.kBrushless);
        backRightMotorTurn = new CANSparkMax(ElectroJendz.BR_TURN_ID, MotorType.kBrushless);
        backRightMotorDrive = new CANSparkMax(ElectroJendz.BR_DRIVE_ID, MotorType.kBrushless);

        frontLeftMotorDrive.setIdleMode(IdleMode.kBrake);
        frontRightMotorDrive.setIdleMode(IdleMode.kBrake);
        backLeftMotorDrive.setIdleMode(IdleMode.kBrake);
        backRightMotorDrive.setIdleMode(IdleMode.kBrake);

        frontLeftMotorTurn.setIdleMode(IdleMode.kBrake);
        frontRightMotorTurn.setIdleMode(IdleMode.kBrake);
        backLeftMotorTurn.setIdleMode(IdleMode.kBrake);
        backRightMotorTurn.setIdleMode(IdleMode.kBrake);

        elevatorMotor = new CANSparkMax(0, MotorType.kBrushless);

        gatherMotorL = new CANSparkMax(0, MotorType.kBrushless);
        gatherMotorR = new CANSparkMax(0, MotorType.kBrushless);
        gatherArmMotor = new CANSparkMax(0, MotorType.kBrushless);
        
        climbMotor = new CANSparkMax(0, MotorType.kBrushless);

    }

    public void run() {
        

    }
    //Assign powers to motors
    public void setSwerveDrivePower(double powerLF, double powerRF, double powerLB, double powerRB) {
        frontLeftMotorDrive.set(powerLF*K.DRV_SwerveDrivePwrScale);
        frontRightMotorDrive.set(powerRF*K.DRV_SwerveDrivePwrScale);
        backLeftMotorDrive.set(-powerLB*K.DRV_SwerveDrivePwrScale);
        backRightMotorDrive.set(-powerRB*K.DRV_SwerveDrivePwrScale);

    }

    //Assign powers to turn motors 
    public void setSwerveDriveTurn(double turnLF, double turnRF, double turnLB, double turnRB) {
        frontLeftMotorTurn.set(turnLF);
        frontRightMotorTurn.set(turnRF);    
        backLeftMotorTurn.set(turnLB);
        backRightMotorTurn.set(turnRB);

    }

    public void setElevatorMotor(double elevate) {
        elevatorMotor.set(elevate);
    }

    public void setGatherMotor(double leftSpeed, double rightSpeed) {
        gatherMotorL.set(leftSpeed);
        gatherMotorR.set(rightSpeed);
    }

    public void setGatherArm(double armGather) {
        gatherArmMotor.set(armGather);
    }

    public void climbMotor(double climb) {
        climbMotor.set(climb);
    }    
       
    


}