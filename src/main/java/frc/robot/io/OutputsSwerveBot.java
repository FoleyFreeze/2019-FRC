package frc.robot.io;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Talon;

public class OutputsSwerveBot extends Outputs {

    private Talon frontRightMotorDrive;
    private TalonSRX frontRightMotorTurn;
    private Talon frontLeftMotorDrive;
    private TalonSRX frontLeftMotorTurn;
    private Talon backRightMotorDrive;
    private TalonSRX backRightMotorTurn;
    private Talon backLeftMotorDrive;
    private TalonSRX backLeftMotorTurn;


    public OutputsSwerveBot() {
        super();

        frontLeftMotorDrive = new Talon(1);
        frontLeftMotorTurn = new TalonSRX(15);
        frontRightMotorDrive = new Talon(0);
        frontRightMotorTurn = new TalonSRX(1);
        backLeftMotorDrive = new Talon(2);
        backLeftMotorTurn = new TalonSRX(9);
        backRightMotorTurn = new TalonSRX(6);
        backRightMotorDrive = new Talon(3);
    }

    public void run() {
        
    }

    public void setSwerveDrivePower(double powerLF, double powerRF, double powerLB, double powerRB) {
        frontLeftMotorDrive.set(limit(powerLF*k.DRV_SwerveDrivePwrScale));
        frontRightMotorDrive.set(limit(powerRF*k.DRV_SwerveDrivePwrScale));
        backLeftMotorDrive.set(limit(powerLB*k.DRV_SwerveDrivePwrScale));
        backRightMotorDrive.set(limit(powerRB*k.DRV_SwerveDrivePwrScale));

    }

    //turn power
    public void setSwerveDriveTurn(double turnLF, double turnRF, double turnLB, double turnRB) {
        frontLeftMotorTurn.set(ControlMode.PercentOutput, turnLF);
        frontRightMotorTurn.set(ControlMode.PercentOutput, turnRF);    
        backLeftMotorTurn.set(ControlMode.PercentOutput, turnLB);
        backRightMotorTurn.set(ControlMode.PercentOutput, turnRB);

    }
}