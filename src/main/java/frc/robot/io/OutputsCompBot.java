package frc.robot.io;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.mil.CurrentLimit;
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

    //anti-fire mode for drive motors
    private CurrentLimit fRDriveMil;
    private CurrentLimit fLDriveMil;
    private CurrentLimit rRDriveMil;
    private CurrentLimit rLDriveMil;

    //anti-fire mode for turn motors
    private CurrentLimit fRTurnMil;
    private CurrentLimit fLTurnMil;
    private CurrentLimit rRTurnMil;
    private CurrentLimit rLTurnMil;

    //anti-fire mode for elevator motors
    private CurrentLimit elevatorMil;

    //anti-fire mode for gather motors
    private CurrentLimit gatherLMil;
    private CurrentLimit gatherRMil;
    private CurrentLimit gatherArmMil;

    //anti-fire mode for climb motors
    private CurrentLimit climbMil;

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
        
        fRDriveMil = new CurrentLimit(40, ElectroJendz.FR_DRIVE_ID, 400);
        fLDriveMil = new CurrentLimit(40, ElectroJendz.FL_DRIVE_ID, 400);
        rLDriveMil = new CurrentLimit(40, ElectroJendz.RL_DRIVE_ID, 400);
        rRDriveMil = new CurrentLimit(40, ElectroJendz.RR_DRIVE_ID, 400);

        fLTurnMil = new CurrentLimit(10, ElectroJendz.FL_TURN_ID, 100);
        fRTurnMil = new CurrentLimit(10, ElectroJendz.FR_TURN_ID, 100);
        rLTurnMil = new CurrentLimit(10, ElectroJendz.RL_TURN_ID, 100);
        rRTurnMil = new CurrentLimit(10, ElectroJendz.RR_TURN_ID, 100);

        //10amp*sec over the 25amp limit
        elevatorMil = new CurrentLimit(10, ElectroJendz.ELE_MotorID, 30);//was 10, 25

        gatherLMil = new CurrentLimit(5, ElectroJendz.GTH_MotorL_ID, 20);
        gatherRMil = new CurrentLimit(5, ElectroJendz.GTH_MotorR_ID, 20);
        gatherArmMil = new CurrentLimit(20, ElectroJendz.GTH_ArmMotorID, 50);

        climbMil = new CurrentLimit(40, ElectroJendz.CLM_MotorID, 400);

        if(!k.DRV_Disable) {
            frontLeftMotorDrive = new CANSparkMax(ElectroJendz.FL_DRIVE_ID, MotorType.kBrushless);
            frontLeftMotorTurn = new CANSparkMax(ElectroJendz.FL_TURN_ID, MotorType.kBrushless);
            frontRightMotorDrive = new CANSparkMax(ElectroJendz.FR_DRIVE_ID, MotorType.kBrushless);
            frontRightMotorTurn = new CANSparkMax(ElectroJendz.FR_TURN_ID, MotorType.kBrushless);
            backLeftMotorDrive = new CANSparkMax(ElectroJendz.RL_DRIVE_ID, MotorType.kBrushless);
            backLeftMotorTurn = new CANSparkMax(ElectroJendz.RL_TURN_ID, MotorType.kBrushless);
            backRightMotorTurn = new CANSparkMax(ElectroJendz.RR_TURN_ID, MotorType.kBrushless);
            backRightMotorDrive = new CANSparkMax(ElectroJendz.RR_DRIVE_ID, MotorType.kBrushless);
            
            frontLeftMotorDrive.getEncoder().setPositionConversionFactor(k.DRV_InchesPRev);
            frontRightMotorDrive.getEncoder().setPositionConversionFactor(k.DRV_InchesPRev);
            backLeftMotorDrive.getEncoder().setPositionConversionFactor(k.DRV_InchesPRev);
            backRightMotorDrive.getEncoder().setPositionConversionFactor(k.DRV_InchesPRev);

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

            frontLeftMotorDrive.setOpenLoopRampRate(k.DRV_MotorDriveRampRate);
            frontRightMotorDrive.setOpenLoopRampRate(k.DRV_MotorDriveRampRate);
            backLeftMotorDrive.setOpenLoopRampRate(k.DRV_MotorDriveRampRate);
            backRightMotorDrive.setOpenLoopRampRate(k.DRV_MotorDriveRampRate);

            frontLeftMotorTurn.setOpenLoopRampRate(k.DRV_MotorTurnRampRate);
            frontRightMotorTurn.setOpenLoopRampRate(k.DRV_MotorTurnRampRate);
            backLeftMotorTurn.setOpenLoopRampRate(k.DRV_MotorTurnRampRate);
            backRightMotorTurn.setOpenLoopRampRate(k.DRV_MotorTurnRampRate);
        }

        if(!k.ELE_Disable){
            elevatorMotor = new CANSparkMax(ElectroJendz.ELE_MotorID, MotorType.kBrushless);
            elevatorMotor.setIdleMode(IdleMode.kBrake);
            elevatorMotor.getEncoder().setPositionConversionFactor(k.ELE_InchesPRev);
        }

        if(!k.GTH_DisableCargo){
            gatherMotorL = new CANSparkMax(ElectroJendz.GTH_MotorL_ID, MotorType.kBrushless);
            gatherMotorR = new CANSparkMax(ElectroJendz.GTH_MotorR_ID, MotorType.kBrushless);
            gatherMotorL.setIdleMode(IdleMode.kBrake);
            gatherMotorR.setIdleMode(IdleMode.kBrake);
        }

        if(!k.GTH_DisableHatch){
            gatherArmMotor = new CANSparkMax(ElectroJendz.GTH_ArmMotorID, MotorType.kBrushed);
        }

        if(!k.CLM_disable){
            climbMotor = new CANSparkMax(ElectroJendz.CLM_MotorID, MotorType.kBrushless);
            climbMotor.setIdleMode(IdleMode.kBrake);
        }
    }

    public void run() {
        super.run();
    }

    public void getEnc(){
        if(!k.DRV_Disable) {
            sense.driveEnc[0] = frontLeftMotorDrive.getEncoder().getPosition();//value in inches
            sense.driveEnc[1] = frontRightMotorDrive.getEncoder().getPosition();//value in inches
            sense.driveEnc[2] = backLeftMotorDrive.getEncoder().getPosition();//value in inches
            sense.driveEnc[3] = backRightMotorDrive.getEncoder().getPosition();//value in inches
        }
        if(!k.ELE_Disable){
            sense.elevatorEncoder = elevatorMotor.getEncoder().getPosition() + k.SEN_ElevatorEncOffset;//value in inches
        }
        if(!k.CLM_disable){
            sense.climberEncoder = climbMotor.getEncoder().getPosition();//value in motor rotations
        }
        
        SmartDashboard.putNumber("Enc FL", sense.driveEnc[0]);
        SmartDashboard.putNumber("Enc FR", sense.driveEnc[1]);
        SmartDashboard.putNumber("Enc RL", sense.driveEnc[2]);
        SmartDashboard.putNumber("Enc RR", sense.driveEnc[3]);
        SmartDashboard.putNumber("Enc Ele", sense.elevatorEncoder);
        SmartDashboard.putNumber("Enc Climb", sense.climberEncoder);
    }

    public void resetEnc(){
        if(!k.DRV_Disable){
            frontLeftMotorDrive.getEncoder().setPosition(0);
            frontRightMotorDrive.getEncoder().setPosition(0);
            backLeftMotorDrive.getEncoder().setPosition(0);
            backRightMotorDrive.getEncoder().setPosition(0);
        }
    }

    //Assign powers to motors
    public void setSwerveDrivePower(double powerLF, double powerRF, double powerLB, double powerRB) {
        double lf = limit(powerLF*k.DRV_SwerveDrivePwrScale/*, fLDriveMil*/);
        double rf = limit(powerRF*k.DRV_SwerveDrivePwrScale/*, fRDriveMil*/);
        double lb = limit(powerLB*k.DRV_SwerveDrivePwrScale/*, rLDriveMil*/);
        double rb = limit(powerRB*k.DRV_SwerveDrivePwrScale/*, rRDriveMil*/);
        
        SmartDashboard.putNumber("Drive_LF",lf);
        SmartDashboard.putNumber("Drive_RF",rf);
        SmartDashboard.putNumber("Drive_LB",lb);
        SmartDashboard.putNumber("Drive_RB",rb);

        frontLeftMotorDrive.set(lf);
        frontRightMotorDrive.set(rf);
        backLeftMotorDrive.set(lb);
        backRightMotorDrive.set(rb);
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
            frontLeftMotorTurn.set(limit(turnLF/*, fLTurnMil*/));
            frontRightMotorTurn.set(limit(turnRF/*, fRTurnMil*/));    
            backLeftMotorTurn.set(limit(turnLB/*, rLTurnMil*/));
            backRightMotorTurn.set(limit(turnRB/*, rRTurnMil*/));
        } else {
            frontLeftMotorTurn.set(0);
            frontRightMotorTurn.set(0);
            backLeftMotorTurn.set(0);
            backRightMotorTurn.set(0);
        }
    }

    public void setElevatorMotor(double elevate) {
        elevatorMotor.set(limit(elevate, elevatorMil));
    }

    public void setGatherMotor(double leftSpeed, double rightSpeed) {
        gatherMotorL.set(limit(leftSpeed/*, gatherLMil*/));
        gatherMotorR.set(limit(rightSpeed/*, gatherRMil*/));
    }

    public void setGatherArm(double armGather) {
        //gatherArmMotor.set(ControlMode.PercentOutput, limit(-armGather, gatherArmMil));
        gatherArmMotor.set(limit(armGather, gatherArmMil)); //ES -armGather
    }

    @Override
    public double getGatherArmCurrent(){
        return sense.pdp.getCurrent(ElectroJendz.GTH_ArmMotorID);
    }

    public void climbMotor(double climb) {
        climbMotor.set(limit(climb, climbMil));
    }    

    @Override
    public void resetEleEnc(){
        elevatorMotor.getEncoder().setPosition(0);
    }

}