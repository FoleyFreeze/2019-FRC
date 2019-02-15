package frc.robot.subsystems;

import frc.robot.util.Angle;

public class AutoDrive extends Component{
    
    int wayPointIndex;
    boolean setPoint;
    double[] rSet;
    double[] thetaSet;
    double endFaceAngle;
    Angle startAngle;
    double[] encValueInit;
    double deltaTheta;
    double rotRate;

    public AutoDrive(){
        startAngle = new Angle();
    }

    public void run(){
        if(!setPoint)return;

        double targetR = rSet[wayPointIndex];
        double targetTheta = thetaSet[wayPointIndex];
        
    }

    public void start(double[] r, double[] theta, double endFaceAngle){
        startAngle.set(sense.robotAngle);
        setPoint = true;
        rSet = r;
        thetaSet = theta;
        deltaTheta = startAngle.subtrahend(endFaceAngle);

        wayPointIndex=0;
        double sumR = 0;
        for(int i = 0; i<r.length-1; i++){
            sumR+=r[i];
        }
        rotRate = deltaTheta/sumR;//DO NOT LET SUMR=0!!!!!!
    }
}