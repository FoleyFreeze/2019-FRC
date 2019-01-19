package frc.robot.io;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Outputs{
    
    private CANSparkMax frontRightMotorDrive;
    private CANSparkMax frontRightMotorTurn;
    private CANSparkMax frontLeftMotorDrive;
    private CANSparkMax frontLeftMotorTurn;
    private CANSparkMax backRightMotorDrive;
    private CANSparkMax backRightMotorTurn;
    private CANSparkMax backLeftMotorDrive;
    private CANSparkMax backLeftMotorTurn;

    private CANSparkMax elevatorMotor;

    private CANSparkMax gatherMotor;
    private CANSparkMax gatherArmMotor;

    private CANSparkMax climbMotor;
    



    public Outputs() {

        frontLeftMotorDrive = new CANSparkMax(0, MotorType.kBrushless);
        frontLeftMotorTurn = new CANSparkMax(0, MotorType.kBrushless);
        frontRightMotorDrive = new CANSparkMax(0, MotorType.kBrushless);
        frontRightMotorTurn = new CANSparkMax(0, MotorType.kBrushless);
        backLeftMotorDrive = new CANSparkMax(0, MotorType.kBrushless);
        backLeftMotorTurn = new CANSparkMax(0, MotorType.kBrushless);
        backRightMotorTurn = new CANSparkMax(0, MotorType.kBrushless);
        backRightMotorDrive = new CANSparkMax(0, MotorType.kBrushless);

        elevatorMotor = new CANSparkMax(0, MotorType.kBrushless);

        gatherMotor = new CANSparkMax(0, MotorType.kBrushless);
        gatherArmMotor = new CANSparkMax(0, MotorType.kBrushless);
        
        climbMotor = new CANSparkMax(0, MotorType.kBrushless);

    }
    public void run() {
        

    }

    public void setSwerveDrivePower(double powerLF, double powerRF, double powerLB, double powerRB) {
        frontLeftMotorDrive.set(powerLF);
        frontRightMotorDrive.set(powerRF);
        backLeftMotorDrive.set(powerLB);
        backRightMotorDrive.set(powerRB);

    }

    public void setSwerveDriveTurn(double turnLF, double turnRF, double turnLB, double turnRB) {
        frontLeftMotorTurn.set(turnLF);
        frontRightMotorTurn.set(turnRF);    
        backRightMotorTurn.set(turnLB);
        backLeftMotorTurn.set(turnRB);

    }

    public void setElevatorMotor(double elevate) {
        elevatorMotor.set(elevate);
    }

    public void setGatherMotor(double gather) {
        gatherMotor.set(gather);
    }

    public void setGatherArm(double armGather) {
        gatherArmMotor.set(armGather);
    }

    public void climbMotor(double climb) {
        climbMotor.set(climb);
    }    
       
    


}