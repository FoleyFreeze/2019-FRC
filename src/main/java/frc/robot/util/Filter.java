import edu.wpi.first.wpilibj.Timer;

public class Filter {

    private double filterConst;
    private boolean autoReset;
    private double timeout = 0.05;
    private double lastVal;
    private double lastTime;

    public Filter(double filterConst, boolean autoReset, double timeout){
        this.filterConst = filterConst;
        this.autoReset = autoReset;
        this.timeout = timeout;
    }

    public double getVal(){
        return lastVal;
    }

    public double run(double value){
        double time = Timer.getFPGATimestamp();
        if(autoReset && time-lastTime > timeout){
            lastVal = value;
        } else {
            lastVal += (value - lastVal) * filterConst;
        }
        lastTime = time;
        return lastVal;
    }

}