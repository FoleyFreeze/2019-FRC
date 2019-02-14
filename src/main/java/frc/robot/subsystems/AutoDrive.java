package frc.robot.subsystems;

public class AutoDrive extends Component{
    
    int wayPointIndex;
    boolean setPoint;
    double[] rSet;
    double[] thetaSet;
    double[] endFaceAngle = new double[20];
    double[] startAngle = new double[20];
    double[] encValueInit;

    public AutoDrive(){
        
    }

    public void run(){
        if(!setPoint)return;

        double targetR = rSet[wayPointIndex];
        double targetTheta = thetaSet[wayPointIndex];
        
    }

    public void start(double[] r, double[] theta, double endFaceAngle){
        if(r.length>startAngle.length)return;
        setPoint = true;
        rSet = r;
        thetaSet = theta;

        wayPointIndex=0;
        double sumR = 0;
        for(int i = 0; i<r.length-1; i++){
            sumR+=r[i];
        }
        for(;;){

        }
    }
}