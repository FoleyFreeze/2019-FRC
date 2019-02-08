package frc.robot.mil;

import frc.robot.util.Filter;

public class Milencoder {

    private String name;
    private double distPerPwrLimit;
    private double minPower;
    private Filter filt;

    private boolean status;

    public Milencoder (String name, double distPerPwrLimit, double minPower) {
        this.name = name;
        this.distPerPwrLimit = distPerPwrLimit;
        this.minPower = minPower;

        filt = new Filter(0.01, false, 0.05);
    }

    public boolean isActive() {
        return status;
    }

    double lastEncVal;
    int failCount = 0;
    public void check(double powerVal, double encVal){
        double ratio = (encVal - lastEncVal) / powerVal;
        if(Math.abs(powerVal) > minPower){
            filt.run(ratio);
            
            if(filt.getVal() > distPerPwrLimit && failCount > 0){
                failCount--;
            } else if (failCount < 10){
                failCount++;
            }

            if(failCount > 7) {
                status = true;
            } else {
                status = false;
            }
        }
    }
}