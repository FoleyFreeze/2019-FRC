package frc.robot.subsystems;

import frc.robot.util.Angle;
import frc.robot.util.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Component{
    public DriveTrain() {
        
    }

    boolean prevDodge = false;
    String mode;

    public void run() {
        if(k.DRV_disable) return;
        
        boolean firstDodge = (in.dodgingL || in.dodgingR) && !prevDodge;
        prevDodge = in.dodgingL || in.dodgingR;

        if(in.dodgingL){
            dodge(firstDodge, -1);
            mode = "Dodge Left";
            SmartDashboard.putString("Mode", mode);
        }else if(in.dodgingR){
            dodge(firstDodge, 1);
            mode = "Dodge Right";
            SmartDashboard.putString("Mode", mode);
        }else if(in.fieldOriented) {
            fieldSwerve(in.xAxisDrive, in.yAxisDrive, in.rotAxisDrive);
            mode = "Field Oriented";
            SmartDashboard.putString("Mode", mode);
        }else{
             swerve(in.xAxisDrive, in.yAxisDrive, in.rotAxisDrive);
             mode = "Regular Swerve";
            SmartDashboard.putString("Mode", mode);
        }
    }

    double[] outR = new double[4];
    double[] outTheta = new double[4];
    double[] outError = new double[4];

    double prevAng = 0;
    double deltaDegSum = 0;
    double dodgeDirX = 0;
    double dodgeDirY = 0;

    public void dodge(boolean firstTime, double turnPower){
        if(firstTime){
            deltaDegSum = 0;
            prevAng = sense.robotAngle.getRad();
            dodgeDirX = Math.cos(prevAng);
            dodgeDirY = Math.sin(prevAng);
            double maxDir = Math.max(dodgeDirX,dodgeDirY);
            if(Math.abs(maxDir) > 1){
                dodgeDirX /= maxDir;
                dodgeDirY /= maxDir;
            }
        }

        double deltaDeg = sense.robotAngle.subRad(prevAng);
        deltaDegSum +=deltaDeg;
        prevAng = sense.robotAngle.getRad();
        if(Math.abs(deltaDegSum) >= 360) turnPower = 0;

        fieldSwerve(dodgeDirX, dodgeDirY, turnPower);
        SmartDashboard.putNumber("Turned Since Dodge:", deltaDegSum);
    }

    //field oriented swerve
    public void fieldSwerve(double xAxis, double yAxis, double rotAxis){
        double theta = sense.robotAngle.getRad();
        double x = xAxis * Math.cos(theta) - yAxis * Math.sin(theta);
        double y = xAxis * Math.sin(theta) + yAxis * Math.cos(theta);
        swerve(x, y, rotAxis);
    }

    boolean prevDriveStraight = false; 
    Angle drvStrSetPnt = new Angle();

    //start timer upon robot startup
    double startTime = Timer.getFPGATimestamp();   

    public void swerve(double xAxis, double yAxis, double rotAxis) {
        //drive straight
        if(in.resetGyro || sense.isDisabled) prevDriveStraight = false;

        // drive straight when we're not turning
        if(rotAxis == 0){
            if(!prevDriveStraight){
                drvStrSetPnt.setDeg(sense.robotAngle);//if first time, set angle to drive at
            } 
            double error = drvStrSetPnt.subDeg(sense.robotAngle);             

            rotAxis = error * k.DRV_SwerveStrKP + sense.deltaRobotAngle * k.DRV_SwerveStrKD;
            prevDriveStraight = true;
        }else{
            prevDriveStraight = false;
        }

       //for each wheel calculate r,theta; power and angle
       for(int i = 0; i < 4; i++){
            double x = k.DRV_WheelLocX[i] - k.DRV_RotCentX;
            double y = k.DRV_WheelLocY[i] - k.DRV_RotCentY;
            
            double h = Math.sqrt(x*x + y*y);
 
            double theta = Math.atan2(y, x);
            theta += Math.PI/2;
 
            double r = k.DRV_SwerveAngRate * h * rotAxis;
 
            double wheelX = r * Math.cos(theta) + xAxis;
            double wheelY = r * Math.sin(theta) + yAxis;
            outR[i] = Math.sqrt(wheelX*wheelX + wheelY*wheelY);
            theta = Math.atan2(wheelY, wheelX) * 180/Math.PI;
            theta += 180;

            outTheta[i] = theta;
       }


        //Normalize
        double maxPwr = Util.absMax(outR);
        if(maxPwr > 1){
            outR[0] /= maxPwr;
            outR[1] /= maxPwr;
            outR[2] /= maxPwr;
            outR[3] /= maxPwr;
        }
         
        boolean parkMode;
        //park if not moving
        double elapsedTime = Timer.getFPGATimestamp() - startTime; 
        if(maxPwr < 0.15 && elapsedTime > k.DRV_WaitForParkTime) {
        
            outR[0] = 0;
            outTheta[0] = 45;
            outR[1] = 0;
            outTheta[1] = 315;
            outR[2] = 0;
            outTheta[2] = 315;
            outR[3] = 0;
            outTheta[3] = 45;
            parkMode = true;
        } else if (maxPwr > 0.15)  { 
            startTime = Timer.getFPGATimestamp(); 
            parkMode = false;
        } else {
            out.setSwerveDrivePower(0,0,0,0);
            out.setSwerveDriveTurn(0,0,0,0);
            parkMode = true;
            return;
        }
        SmartDashboard.putBoolean("Park Mode?", parkMode);
        SmartDashboard.putNumberArray("TargetAngles", outTheta);

        //pid to target angle (theta)
        for(int i=0; i<4; i++){

            double error = sense.angles[i].subDeg(outTheta[i]);

            //pick shortest path
            if(Math.abs(error) > 90){
                if(error > 0) error -= 180;
                else error += 180;

                outR[i] = -outR[i];
            }
            SmartDashboard.putNumber("Error " + i, error);
            
            double anglePower = k.DRV_SwerveAngKP * error;
            outError[i] = Math.max(Math.min(k.DRV_SwerveMaxAnglePwr, anglePower), -k.DRV_SwerveMaxAnglePwr);
        }

        out.setSwerveDrivePower(outR[0], outR[1], outR[2], outR[3]);
        out.setSwerveDriveTurn(outError[0], outError[1], outError[2], outError[3]);
        SmartDashboard.putNumberArray("Drive Power", outR);
        SmartDashboard.putNumberArray("Turn Power", outError);
    }
    public void cameraDrive(double distance, double angleTo, double angleOf){ 
         double X = (angleOf - 90) * k.DRV_ofTargetAngleKP;
         double Y = distance * k.DRV_targetDistanceKP;
         double R = angleTo * k.DRV_toTargetAngleKP;
         swerve(X, Y, R);
    }
}