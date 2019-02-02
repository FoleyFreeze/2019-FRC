package frc.robot.subsystems;

import frc.robot.io.K;
import frc.robot.util.Angle;
import frc.robot.util.Util;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrain extends Component{
    public DriveTrain() {
        
    }

    public void run() {
        if(in.fieldOriented) {
            fieldSwerve(in.xAxisDrive, in.yAxisDrive, in.rotAxisDrive);
        }else{
             swerve(in.xAxisDrive, in.yAxisDrive, in.rotAxisDrive);
        }
    }

    double[] outR = new double[4];
    double[] outTheta = new double[4];
    double[] outError = new double[4];

    //field oriented swerve
    public void fieldSwerve(double xAxis, double yAxis, double rotAxis){
        double theta = Math.atan2(yAxis, xAxis) * 180 / Math.PI;
        double r = Math.sqrt(xAxis * xAxis + yAxis * yAxis);
        theta = -(sense.robotAngle.add(r));
        double x = r * Math.cos(theta / 180 * Math.PI);
        double y = r * Math.sin(theta / 180 * Math.PI);
        swerve(x, y, in.rotAxisDrive);
    }

    boolean driveStraight = false; 
    Angle drvStrSetPnt = new Angle();

    //start timer upon robot startup
    double startTime = Timer.getFPGATimestamp();   

    public void swerve(double xAxis, double yAxis, double rotAxis) {
        if(rotAxis == 0){
            if(!driveStraight){
                drvStrSetPnt.set(sense.robotAngle);
                drvStrSetPnt = sense.robotAngle;
            } 
            double error = drvStrSetPnt.sub(sense.robotAngle);
            
            rotAxis = error * K.DRV_SwerveStrKP;
            driveStraight = true;
        }else{
            driveStraight = false;
        }

       //for each wheel calculate r,theta; power and angle
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
       }


        //Normalize
        double maxPwr = Util.absMax(outR);
        if(maxPwr > 1){
            outR[0] /= maxPwr;
            outR[1] /= maxPwr;
            outR[2] /= maxPwr;
            outR[3] /= maxPwr;
        }
         
        //park if not moving
         double elapsedTime = Timer.getFPGATimestamp() - startTime; 
        if(maxPwr < 0.15 && elapsedTime > K.DRV_WaitForParkTime) {
        
            outR[0] = 0;
            outTheta[0] = 45;
            outR[1] = 0;
            outTheta[1] = 315;
            outR[2] = 0;
            outTheta[2] = 315;
            outR[3] = 0;
            outTheta[3] = 45;
        } else if (maxPwr > 0.15)  { 
            startTime = Timer.getFPGATimestamp(); 
        } else {
            out.setSwerveDrivePower(0,0,0,0);
            out.setSwerveDriveTurn(0,0,0,0);
            return;
        }


        //pid to target angle (theta)
        for(int i=0; i<4; i++){

            double error = sense.angles[i].sub(outTheta[i]);

            //pick shortest path
            if(Math.abs(error) > 90){
                if(error > 0) error -= 180;
                else error += 180;

                outR[i] = -outR[i];
            }
            
            double anglePower = K.DRV_SwerveAngKP * error;
            outError[i] = Math.max(Math.min(K.DRV_SwerveMaxAnglePwr, anglePower), -K.DRV_SwerveMaxAnglePwr);
        }

        //out.swerveDriveAngle(sense.angles, relEnc, outTheta, outR);
        out.setSwerveDrivePower(outR[0], outR[1], outR[2], outR[3]);
        out.setSwerveDriveTurn(outError[0], outError[1], outError[2], outError[3]);
    }
}