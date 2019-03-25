package frc.robot.subsystems;

import frc.robot.subsystems.vision.VisionData;
import frc.robot.util.Angle;
import frc.robot.util.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Component{
    public boolean autoShoot;
    
    public DriveTrain() {
        
    }

    boolean prevDodge = false;
    String mode;

    public void run() {
        if(k.DRV_Disable) return;
        
        autoShoot = false;
        if(in.autoDrive){
            autoDrive();
            return;
        }
        
        if(selectCameraDrive()) return;

        if(in.climb){
            swerve(0,-k.CLM_DrivePower,0);
            return;
        }

        if(!k.DRV_DisableAutoOrient && in.autoOrientRobot && Math.abs(in.robotOrientation) < 360){
            double rotPower = pidOrient();
            if(in.fieldOriented)fieldSwerve(in.xAxisDrive, in.yAxisDrive, rotPower);
            else swerve(in.xAxisDrive, in.yAxisDrive, rotPower);
            return;
        }

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

    private void autoDrive(){

        //if path is complete, then camera drive
        if(autoDriving.pathComplete) {
            if(!selectCameraDrive()){
                if(in.fieldOriented){
                    fieldSwerve(in.xAxisDrive, in.yAxisDrive, pidOrient());
                } else {
                    swerve(in.xAxisDrive, in.yAxisDrive, pidOrient());
                }
            }
        //if there is a target point, PID towards it
        } else if(autoDriving.targetPoint != null){

            //calc powers for X and Y based on target point and rse
            double distX = autoDriving.targetPoint.x - rse.x;
            double distY = autoDriving.targetPoint.y - rse.y;

            //PID and limit magnitude
            /*
            double r = Math.sqrt(distX*distX + distY * distY);
            double rPwr = Util.limit(r * k.AD_AutoDriveKP, k.AD_MaxPower);
            double theta = Math.atan2(distX,distY);

            double autoX = rPwr * Math.cos(theta);
            double autoY = rPwr * Math.sin(theta);
            */
            double autoX = Util.limit(distX * k.AD_AutoDriveKP, k.AD_MaxPower);
            double autoY = Util.limit(distY * k.AD_AutoDriveKP, k.AD_MaxPower);

            //get rot power form pidOrient
            double autoRot = pidOrient();
            
            //field swerve
            fieldSwerve(autoX, autoY, autoRot);
        }
    }

    private boolean selectCameraDrive(){
        if(in.visionTargetHigh) {
            VisionData vd = view.getLastVisionTargetHigh();
            if(vd != null && Timer.getFPGATimestamp() - vd.timeStamp < k.CAM_ExpireTime) {
                cameraDrive(vd);
                return true;
            } 
        } else if(in.visionTargetLow){
            VisionData vd = view.getLastVisionTargetLow();
            if(vd != null && Timer.getFPGATimestamp() - vd.timeStamp < k.CAM_ExpireTime) {
                cameraDrive(vd);
                return true;
            }
        } else if(in.visionCargo) {
            VisionData vd = view.getLastCargo();
            if(vd != null && Timer.getFPGATimestamp() - vd.timeStamp < k.CAM_ExpireTime) {
                cameraDrive(vd);
                return true;
            }
        }

        return false;
    }

    private double pidOrient(){
        //PID rotation until robot angle equals robotOrientation
        double angleErr = sense.robotAngle.subDeg(in.robotOrientation);
        SmartDashboard.putNumber("autoRotateError", angleErr);
        double rotPower = angleErr * k.DRV_AutoRotateKP;
        rotPower = Math.max(-k.DRV_AutoRotatePwr, Math.min(k.DRV_AutoRotatePwr, rotPower));
        return rotPower;
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
        double theta = -sense.robotAngle.getRad();
        double x = xAxis * Math.cos(theta) - yAxis * Math.sin(theta);
        double y = xAxis * Math.sin(theta) + yAxis * Math.cos(theta);
        swerve(x, y, rotAxis);
    }

    boolean prevDriveStraight = false;
    double driveStraightTime = 0; 
    Angle drvStrSetPnt = new Angle();

    //start timer upon robot startup
    double startTime = Timer.getFPGATimestamp(); 
    
    public boolean parkMode;

    public void swerve(double xAxis, double yAxis, double rotAxis){
        swerve(xAxis, yAxis, rotAxis, k.DRV_RotCentX, k.DRV_RotCentY);
    }
    
    public void swerve(double xAxis, double yAxis, double rotAxis, double xRotCtr, double yRotCtr) {
        //drive straight
        if(in.resetGyro || sense.isDisabled) prevDriveStraight = false;

        // drive straight when we're not turning
        if(rotAxis == 0){
            if(!prevDriveStraight){
                driveStraightTime = Timer.getFPGATimestamp() + k.DRV_DriveStraightDelay;
            } else if(!prevDriveStraight && Timer.getFPGATimestamp() > driveStraightTime){
                drvStrSetPnt.setDeg(sense.robotAngle);//if first time, set angle to drive at
                prevDriveStraight = true;
            } else {
                double error = drvStrSetPnt.subDeg(sense.robotAngle);             
                rotAxis = error * k.DRV_SwerveStrKP + sense.deltaRobotAngle * k.DRV_SwerveStrKD;
            }
            
        }else{
            prevDriveStraight = false;
        }

       //for each wheel calculate r,theta; power and angle
       for(int i = 0; i < 4; i++){
            double x = k.DRV_WheelLocX[i] - xRotCtr;
            double y = k.DRV_WheelLocY[i] - yRotCtr;
            
            double h = Math.sqrt(x*x + y*y);
 
            double theta = Math.atan2(y, x);
            
            //degrees to radians
            theta += Math.PI/2;
 
            double r = k.DRV_SwerveAngRate * h * rotAxis;
 
            double wheelX = r * Math.cos(theta) + xAxis;
            double wheelY = r * Math.sin(theta) + yAxis;
            outR[i] = Math.sqrt(wheelX*wheelX + wheelY*wheelY);

            //radians to degrees
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
         
        //park if not moving
        if(in.fallingEdgeClimb) startTime = 0; //goto park immediately after climb
        double elapsedTime = Timer.getFPGATimestamp() - startTime; 
        if(maxPwr < 0.02 && elapsedTime > k.DRV_WaitForParkTime) {
        
            outR[0] = 0;
            outTheta[0] = 315;
            outR[1] = 0;
            outTheta[1] = 45;
            outR[2] = 0;
            outTheta[2] = 45;
            outR[3] = 0;
            outTheta[3] = 315;
            parkMode = true;
        } else if (maxPwr > 0.02)  { 
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
        double powerFactor = Util.interpolate(k.DRV_EleHeightAxis, k.DRV_PowerTable, sense.elevatorEncoder);

        out.setSwerveDrivePower(outR[0]*powerFactor, outR[1]*powerFactor, outR[2]*powerFactor, outR[3]*powerFactor);
        out.setSwerveDriveTurn(outError[0], outError[1], outError[2], outError[3]);
        SmartDashboard.putNumberArray("Drive Power", outR);
        SmartDashboard.putNumberArray("Turn Power", outError);
    }

    public void cameraDrive(VisionData vd) { 
        if(in.visionCargo){

        }else{
            //get angle and distance from vd
            
            //turn angle into a x distance
            double distX = (vd.distance + 16) * Math.tan(Angle.toRad(vd.angleTo));
            double distY = vd.distance - k.DRV_CamTargetY0;

            //PID x and y powers to x y distances
            double xPower = Util.limit(distX * k.DRV_TargetDistanceKP, k.DRV_CamDriveMaxPwr_X);
            double yPower = Util.limit(distY * k.DRV_TargetDistanceKP, k.DRV_CamDriveMaxPwr_Y);
            double rotPower = pidOrient();

            //normalize y power based on applied x power
            yPower *= 1 - Math.abs(xPower) / k.DRV_CamDriveMaxPwr_X;

            //call swerve w/ x and y powers & auto rotate power
            swerve(xPower, yPower, rotPower);

            //determine if we should autoShoot
            autoShoot = Math.abs(distX) < k.DRV_CamDistShootX && distY < k.DRV_CamDistShootY;
        }
    }

}