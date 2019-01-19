package frc.robot.subsystems;

import frc.robot.io.K;
import frc.robot.util.Util;

public class DriveTrain extends Component{
    public DriveTrain() {
        
    }

    public void run() {
        swerve(in.xAxisDrive, in.yAxisDrive, in.rotAxisDrive);
    }

    double[] outR = new double[4];
    double[] outTheta = new double[4];

    public void swerve(double xAxis, double yAxis, double rotAxis) {

       if(Math.abs(xAxis) < 0.05) xAxis = 0; 
       if(Math.abs(yAxis) < 0.05) yAxis = 0; 

       if(Math.abs(xAxis) < 0.2 && Math.abs(yAxis) < 0.2) {
        xAxis = 0;
        yAxis = 0;
       }

       if(Math.abs(rotAxis) < 0.2) rotAxis = 0;
       
       for(int i = 0; i < 4; i++){
            double x = K.DRV_WheelLocX[i] - K.DRV_RotCentX;
            double y = K.DRV_WheelLocY[i] - K.DRV_RotCentY;
            
            double h = Math.sqrt(x*x + y*y);
 
            double theta = Math.atan2(y, x);
            theta += Math.PI/2;
 
            double r = K.DRV_SwerveAngRate * h * rotAxis;
 
            double wheelX = r * Math.cos(theta) + xAxis;
            double wheelY = r * Math.sin(theta) + yAxis;
            outR[i] = Math.sqrt(wheelX*wheelX + wheelY*wheelY);
            theta = Math.atan2(wheelY, wheelX) * 180/Math.PI;
            theta += 180;

            double measuredValue = sense.wheelAngles[i].getDistance()/K.DRV_CountsPerDegree;
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
            
            double anglePower = K.DRV_SwerveAngKP * error;
            outTheta[i] = Math.max(Math.min(K.DRV_SwerveMaxAnglePwr, anglePower), -K.DRV_SwerveMaxAnglePwr);
       }

       //Normalize
       double maxPwr = Util.absMax(outR);
       if(maxPwr > 1){
           outR[0] /= maxPwr;
           outR[1] /= maxPwr;
           outR[2] /= maxPwr;
           outR[3] /= maxPwr;
       }

       out.setSwerveDrivePower(outR[0], outR[1], outR[2], outR[3]);
       out.setSwerveDriveTurn(outTheta[0], outTheta[1], outTheta[2], outTheta[3]);
    }
}