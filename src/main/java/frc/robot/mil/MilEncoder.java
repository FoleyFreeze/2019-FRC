package frc.robot.mil;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Component;
import frc.robot.util.Angle;
import frc.robot.util.Filter;

public class MilEncoder extends Component{

    private String name;
    private double distPerPwrLimit;
    private double minPower;
    private Filter filt;

    private boolean status;

    public MilEncoder (String name, double distPerPwrLimit, double minPower) {
        this.name = name;
        this.distPerPwrLimit = distPerPwrLimit;
        this.minPower = minPower;

        filt = new Filter(0.1);
        filt.setVal(distPerPwrLimit*2); //start higher than the limit
    }

    public boolean isActive() {
        if(k.MIL_Disabled) return false;
        //SALwasHere-SmartDashboard.putBoolean("Mil " + name, status);
        //SALwasHere-SmartDashboard.putNumber("Mil val " + name, filt.getVal());
        return status;
    }

    private double lastEncVal;
    private int failCount = 0;

    //works for angle wrap around
    public void check(double powerVal, Angle angle){
        double diff = angle.subDeg(lastEncVal);
        lastEncVal = angle.getDeg();
        checkInternal(powerVal, diff);
    }

    //works for normal encoders
    public void check(double powerVal, double encVal) {
        double diff = encVal - lastEncVal;
        lastEncVal = encVal;
        checkInternal(powerVal, diff);
    } 

    private void checkInternal(double powerVal, double encDiff){
        double ratio = Math.abs(encDiff) / powerVal;

        //only filter the ratio when we are above some min power level
        if(Math.abs(powerVal) > minPower){
            filt.run(ratio);
            
            //check if our filtered ratio meets requirements
            if(filt.getVal() < distPerPwrLimit && failCount > 0){
                failCount--;
            } else if (failCount < 10){
                failCount++;
            }

            //if failted 7 of the last 10 times, set the status to faulted
            if(failCount > 7) {
                status = true;
            } else {
                status = false;
            }
        }
    }
}