package frc.robot.subsystems;

import frc.robot.io.K;

public class DriveTrain extends Component{
    public DriveTrain() {
        
    }

    double[] outR = new double[4];
    double[] outTheta = new double[4];

    public void run() {

       if(Math.abs(in.xAxisDrive) < 0.05) in.xAxisDrive = 0; 
       if(Math.abs(in.yAxisDrive) < 0.05) in.yAxisDrive = 0; 

       if(Math.abs(in.xAxisDrive) < 0.2 && Math.abs(in.yAxisDrive) < 0.2) {
        in.xAxisDrive = 0;
        in.yAxisDrive = 0;
       }

       if(Math.abs(in.rotAxisDrive) < 0.2) in.rotAxisDrive = 0;
       
       for(int i = 0; i < 4; i++){
            double x = K.wheelXLoc[i] - K.rotCentX;
            double y = K.wheelYLoc[i] - K.rotCentY;
            
            double h = Math.sqrt(x*x + y*y);
 
            double theta = Math.atan2(y, x);
            theta += Math.PI/2;
 
            double r = K.swerveAngRate * h * in.rotAxisDrive;
 
            double wheelX = r * Math.cos(theta) + in.xAxisDrive;
            double wheelY = r * Math.sin(theta) + in.yAxisDrive;
            outR[i] = Math.sqrt(wheelX*wheelX + wheelY*wheelY);
            theta = Math.atan2(wheelY, wheelX) * 180/Math.PI;
            theta += 180;

            double measuredValue = sense.wheelAngles[i].getDistance()/K.countsPerDegree;
            measuredValue = measuredValue % 360;
            if(measuredValue < 0) measuredValue += 360;
            
            double error = measuredValue - theta;
            if(error > 180) error -= 360;
            if(error < -180) error += 360;

            if(Math.abs(error) > 90){
                if(error > 0) error -= 180;
                else error += 180;

                outR[i] = -outR[i];
            }
            
            double anglePower = K.swerveAngKP * error;
            outTheta[i] = Math.max(Math.min(0.5, anglePower), -0.5);

            
            


       }
    }
}