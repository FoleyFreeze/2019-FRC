package frc.robot.io;

import frc.robot.subsystems.Component;

public abstract class Outputs extends Component {
 

    public Outputs() {
        
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
       
    protected double limit(double value) {
        if(in.pitMode){
            value = Math.min(Calibrations.OUT_PitModeLimit, Math.max(-Calibrations.OUT_PitModeLimit, value));
        } 
        return value;
    }


}