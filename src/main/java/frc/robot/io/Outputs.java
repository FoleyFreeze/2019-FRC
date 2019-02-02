package frc.robot.io;

import edu.wpi.first.wpilibj.Solenoid;

public abstract class Outputs {
 
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
    
    public void suction(boolean enable) {
        gatherSolenoid1.set(enable);
        gatherSolenoid2.set(enable);
    }
    


}