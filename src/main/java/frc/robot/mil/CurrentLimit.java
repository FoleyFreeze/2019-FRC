package frc.robot.mil;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Component;
import frc.robot.util.Filter;

public class CurrentLimit extends Component{
    private double limitTime;
    private double limitCurr;
    private int channel;
    private boolean active;
    private Filter filCurr;
    private double actTime;

    public CurrentLimit(double limitTime, int channel, double limitCurr){
        this.limitTime = limitTime;
        this.limitCurr = limitCurr;
        if(channel == 20) channel = 0;
        this.channel = channel;
        filCurr = new Filter(k.MIL_CurrFilt*0.02);
    }

    public void run(){
        double current;
        try{
         current = sense.pdp.getCurrent(channel);
        } catch (Error e){
            System.out.println(e.getMessage());
            //SALwasHere-SmartDashboard.putNumber("TimeOfPDPError", Timer.getFPGATimestamp());
            current = 0;
        }
        if(current>limitCurr){
            //dont actually filter, but update the value in the in the filter based on Amp-seconds over the limit
            double sumCurrent = filCurr.getVal() + (current-limitCurr)*sense.dt;
            filCurr.setVal(sumCurrent);
        }else{
            //use the low-pass filter to decay any overcurrent values when we are within the limit
            filCurr.run(0);
        }
        if(filCurr.getVal() > limitTime){
            active = true;
            actTime = Timer.getFPGATimestamp();
        }else {
            //set active false only if enough time has passed
            if(Timer.getFPGATimestamp()-actTime > k.MIL_CurrResetTime) active = false;
        }
    }
    
    public boolean isActive(){
        if(k.MIL_CLDisabled) return false;
        return active;
    }
}