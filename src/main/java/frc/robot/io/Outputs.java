package frc.robot.io;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.mil.CurrentLimit;
import frc.robot.subsystems.Component;

public abstract class Outputs extends Component {
 
    private Solenoid gatherSolenoid1;
    private Solenoid gatherSolenoid2;



    public Outputs() {
        gatherSolenoid1 = new Solenoid(1);
        gatherSolenoid2 = new Solenoid(2);
    }

    public void run() {
        
    }

    //Assign powers to motors
    public void setSwerveDrivePower(double powerLF, double powerRF, double powerLB, double powerRB) {

    }

    //Assign powers to turn motors 
    public void setSwerveDriveTurn(double turnLF, double turnRF, double turnLB, double turnRB) {
        
    }

    public void setElevatorMotor(double elevate) {
        
    }

    public void setGatherMotor(double leftSpeed, double rightSpeed) {
       
    }

    public void setGatherArm(double armGather) {
       
    }

    public void climbMotor(double climb) {
        
    }    

    public void getEnc(){
        
    }

    public void resetEnc(){
        
    }
       
    protected double limit(double value, CurrentLimit cLIM) {
        if(in.pitMode){
            value = Math.min(k.OUT_PitModeLimit, Math.max(-k.OUT_PitModeLimit, value));
        } 
        if(sense.isDisabled){
            value = 0;
        }
        cLIM.run();
        if(cLIM.isActive()){
            value = Math.min(k.OUT_PitModeLimit, Math.max(-k.OUT_PitModeLimit, value));
        }

        return value;
    }

    protected double limit(double value) {
        if(in.pitMode){
            value = Math.min(k.OUT_PitModeLimit, Math.max(-k.OUT_PitModeLimit, value));
        } 
        if(sense.isDisabled){
            value = 0;
        }
        return value;
    }

    public void suction(boolean enable) {
        gatherSolenoid1.set(enable);
        gatherSolenoid2.set(enable);
    }

    public double getGatherArmCurrent(){
        return 0;
    }
}