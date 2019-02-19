package frc.robot.io;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.mil.MilEncoder;

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


    //output mils
    private MilEncoder milTurnFL;
    private MilEncoder milTurnFR;
    private MilEncoder milTurnRL;
    private MilEncoder milTurnRR;

    public OutputsCompBot() {
        super();

        milTurnFL = new MilEncoder("FL_Turn", 20, 0.1);
        milTurnFR = new MilEncoder("FR_Turn", 20, 0.1);
        milTurnRL = new MilEncoder("RL_Turn", 20, 0.1);
        milTurnRR = new MilEncoder("RR_Turn", 20, 0.1);

        if(!k.DRV_disable) {
            frontLeftMotorDrive = new CANSparkMax(ElectroJendz.FL_DRIVE_ID, MotorType.kBrushless);
            frontLeftMotorTurn = new CANSparkMax(ElectroJendz.FL_TURN_ID, MotorType.kBrushless);
            frontRightMotorDrive = new CANSparkMax(ElectroJendz.FR_DRIVE_ID, MotorType.kBrushless);
            frontRightMotorTurn = new CANSparkMax(ElectroJendz.FR_TURN_ID, MotorType.kBrushless);
            backLeftMotorDrive = new CANSparkMax(ElectroJendz.BL_DRIVE_ID, MotorType.kBrushless);
            backLeftMotorTurn = new CANSparkMax(ElectroJendz.BL_TURN_ID, MotorType.kBrushless);
            backRightMotorTurn = new CANSparkMax(ElectroJendz.BR_TURN_ID, MotorType.kBrushless);
            backRightMotorDrive = new CANSparkMax(ElectroJendz.BR_DRIVE_ID, MotorType.kBrushless);
        

            if (k.OUT_DriveBrakeMode){
                frontLeftMotorDrive.setIdleMode(IdleMode.kBrake);
                frontRightMotorDrive.setIdleMode(IdleMode.kBrake);
                backLeftMotorDrive.setIdleMode(IdleMode.kBrake);
                backRightMotorDrive.setIdleMode(IdleMode.kBrake);
            } else {
                frontLeftMotorDrive.setIdleMode(IdleMode.kCoast);
                frontRightMotorDrive.setIdleMode(IdleMode.kCoast);
                backLeftMotorDrive.setIdleMode(IdleMode.kCoast);
                backRightMotorDrive.setIdleMode(IdleMode.kCoast);
            }

            frontLeftMotorTurn.setIdleMode(IdleMode.kBrake);
            frontRightMotorTurn.setIdleMode(IdleMode.kBrake);
            backLeftMotorTurn.setIdleMode(IdleMode.kBrake);
            backRightMotorTurn.setIdleMode(IdleMode.kBrake);
        }

        if(!k.ELE_disable){
            elevatorMotor = new CANSparkMax(ElectroJendz.ELE_MotorID, MotorType.kBrushless);
            elevatorMotor.setIdleMode(IdleMode.kBrake);
        }

        if(!k.GTH_disableBall){
            gatherMotorL = new CANSparkMax(ElectroJendz.GTH_MotorL_ID, MotorType.kBrushless);
            gatherMotorR = new CANSparkMax(ElectroJendz.GTH_MotorR_ID, MotorType.kBrushless);
        }

        if(!k.GTH_disableDisk){
            gatherArmMotor = new CANSparkMax(ElectroJendz.GTH_ArmMotorID, MotorType.kBrushless);
        }

        if(!k.CLM_disable){
            climbMotor = new CANSparkMax(ElectroJendz.CLM_MotorID, MotorType.kBrushless);
        }
    }

    public void run() {
        
    }

    public void getEnc(){
        if(!k.DRV_disable) {
            sense.driveEnc[0] = frontLeftMotorDrive.getEncoder().getPosition()*k.DRV_InchesPRev;//value in inches
            sense.driveEnc[1] = frontRightMotorDrive.getEncoder().getPosition()*k.DRV_InchesPRev;//value in inches
            sense.driveEnc[2] = backLeftMotorDrive.getEncoder().getPosition()*k.DRV_InchesPRev;//value in inches
            sense.driveEnc[3] = backRightMotorDrive.getEncoder().getPosition()*k.DRV_InchesPRev;//value in inches
        }
        if(!k.ELE_disable){
            sense.elevatorEncoder = elevatorMotor.getEncoder().getPosition()*k.ELE_InchesPRev;//value in inches
        }
        
        SmartDashboard.putNumber("Enc FL", sense.driveEnc[0]);
        SmartDashboard.putNumber("Enc FR", sense.driveEnc[1]);
        SmartDashboard.putNumber("Enc RL", sense.driveEnc[2]);
        SmartDashboard.putNumber("Enc RR", sense.driveEnc[3]);
        SmartDashboard.putNumber("Enc Ele", sense.elevatorEncoder);
    }

    public void resetEnc(){
        if(!k.DRV_disable){
            frontLeftMotorDrive.getEncoder().setPosition(0);
            frontRightMotorDrive.getEncoder().setPosition(0);
            backLeftMotorDrive.getEncoder().setPosition(0);
            backRightMotorDrive.getEncoder().setPosition(0);
        }
    }

    //Assign powers to motors
    public void setSwerveDrivePower(double powerLF, double powerRF, double powerLB, double powerRB) {
        frontLeftMotorDrive.set(limit(powerLF*k.DRV_SwerveDrivePwrScale));
        frontRightMotorDrive.set(limit(powerRF*k.DRV_SwerveDrivePwrScale));
        backLeftMotorDrive.set(limit(-powerLB*k.DRV_SwerveDrivePwrScale));
        backRightMotorDrive.set(limit(-powerRB*k.DRV_SwerveDrivePwrScale));
    }

    //Assign powers to turn motors 
    public void setSwerveDriveTurn(double turnLF, double turnRF, double turnLB, double turnRB) {

        milTurnFL.check(turnLF, sense.angles[0]);
        milTurnFR.check(turnRF, sense.angles[1]);
        milTurnRL.check(turnLB, sense.angles[2]);
        milTurnRR.check(turnRB, sense.angles[3]);
 
        int count = 0;
        //TODO - stop moving the wheel that isn't moving!
        if(milTurnFL.isActive()) count++;
        if(milTurnFR.isActive()) count++;
        if(milTurnRL.isActive()) count++;
        if(milTurnRR.isActive()) count++;

        //attempt to still drive if one encoder is bad
        if(count < 2) {
            frontLeftMotorTurn.set(limit(turnLF));
            frontRightMotorTurn.set(limit(turnRF));    
            backLeftMotorTurn.set(limit(turnLB));
            backRightMotorTurn.set(limit(turnRB));
        } else {
            frontLeftMotorTurn.set(0);
            frontRightMotorTurn.set(0);
            backLeftMotorTurn.set(0);
            backRightMotorTurn.set(0);
        }
    }

    public void setElevatorMotor(double elevate) {
        elevatorMotor.set(limit(elevate));
    }

    public void setGatherMotor(double leftSpeed, double rightSpeed) {
        gatherMotorL.set(limit(leftSpeed));
        gatherMotorR.set(limit(rightSpeed));
    }

    public void setGatherArm(double armGather) {
        gatherArmMotor.set(limit(armGather));
    }

    public void climbMotor(double climb) {
        climbMotor.set(limit(climb));
    }    

}