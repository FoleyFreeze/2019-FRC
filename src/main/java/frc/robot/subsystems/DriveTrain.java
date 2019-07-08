package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.ControlBoard.NearFarCargo;
import frc.robot.subsystems.vision.VisionData;
import frc.robot.util.Angle;
import frc.robot.util.Filter;
import frc.robot.util.Util;
import frc.robot.subsystems.autodrive.Point;

public class DriveTrain extends Component {
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

        if(!k.DRV_DisableAutoOrient && in.autoOrientRobot && !sense.isAuto && Math.abs(in.robotOrientation) < 360){
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

            //get x,y drive Point (power) from autoDrive
            Point p = autoDriving.getDrivePower();
            
            //double autoX = Util.limit(distX * k.AD_AutoDriveKP, k.AD_MaxPower);
            //double autoY = Util.limit(distY * k.AD_AutoDriveKP, k.AD_MaxPower);

            //get rot power form pidOrient, scale rotate power by autodrive power
            double autoRot;
            if(autoDriving.enableAutoTurn && !autoDriving.startingHab2) autoRot = pidOrient() * autoDriving.powerLim;
            else autoRot = 0;
            
            //field swerve
            fieldSwerve(p.x, p.y, autoRot);
        } else {
            swerve(0,0,0);
        }
    }

    private boolean selectCameraDrive(){
        //if we already gathered or shot the object
        if(in.latchReady){
            if(sense.hasCargo){
                if(in.fieldOriented) fieldSwerve(in.xAxisDrive,in.yAxisDrive,in.rotAxisDrive);
                else swerve(in.xAxisDrive,in.yAxisDrive,in.rotAxisDrive);
            } else {
                //force negative Y in order to force reversing
                //swerve(in.xAxisDrive, Math.max(in.yAxisDrive,0.3), pidOrient());
                swerve(0, Math.max(0,0.3), pidOrient());
            }
            
            return true;
        }

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
        if(autoDriving.startingHab2 && sense.isAuto) return 0;
        
        //PID rotation until robot angle equals robotOrientation
        double angleErr = sense.robotAngle.subDeg(in.robotOrientation);
        SmartDashboard.putNumber("autoRotateError", angleErr);
        double rotPower = angleErr * k.DRV_AutoRotateKP + sense.deltaRobotAngle * k.DRV_AutoRotateKD;
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
        double powerFactor = Util.interpolate(k.DRV_EleHeightAxis, k.DRV_PowerTable, sense.elevatorEncoder);
        double maxPwr = Util.absMax(outR);
        if(maxPwr > powerFactor){   //was 1 instead of powerFactor
            outR[0] = outR[0] / maxPwr * powerFactor;
            outR[1] = outR[1] / maxPwr * powerFactor;
            outR[2] = outR[2] / maxPwr * powerFactor;
            outR[3] = outR[3] / maxPwr * powerFactor;
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
            
            if(k.DRV_PIDOnSpark){
                outError[i] = -error;
            } else {
                double anglePower = k.DRV_SwerveAngKP * error;
                outError[i] = Math.max(Math.min(k.DRV_SwerveMaxAnglePwr, anglePower), -k.DRV_SwerveMaxAnglePwr);    
            }

        }

        out.setSwerveDrivePower(outR[0], outR[1], outR[2], outR[3]);

        if(k.DRV_PIDOnSpark){
            out.setSwerveDriveTurnAngle(outError[0], outError[1], outError[2], outError[3]);
        } else {
            out.setSwerveDriveTurn(outError[0], outError[1], outError[2], outError[3]);
        }
        
        SmartDashboard.putNumberArray("Drive Power", outR);
        SmartDashboard.putNumberArray("Turn Power", outError);
    }

    private Filter lpf = new Filter(0.5, true, 0.06, 0);

    public void cameraDrive(VisionData vd) { 
        if(in.visionCargo){
            double deltaAngle = sense.robotAngle.subDeg(vd.robotAngle) + vd.angleTo;
            double dist = Math.max(vd.distance, 0);
            double turnPwr = Util.limit(deltaAngle * k.DRV_CamCargoThetaKP + sense.deltaRobotAngle *k.DRV_CamCargoThetaKD , k.DRV_CamCargoPwrLim);
            //PID orient for ref: double rotPower = angleErr * k.DRV_AutoRotateKP + sense.deltaRobotAngle * k.DRV_AutoRotateKD;

            double distPwr = Util.limit(dist * k.DRV_CamCargoDistKP, k.DRV_CamCargoPwrLim);
            //if(gatherer.scorpioActive()){
            //    swerve(0,0,0);
            //} else {
                swerve(0,distPwr,turnPwr);
            //}
            //if(k.SCR_ScorpioSelected) {
            //    autoShoot = vd.distance < 15 && Math.abs(deltaAngle) < 5;
            //} else {
                autoShoot = true;
            //}
        } else { //vision target is not cargo

            double vOffset;
            double xErrLimit;

            if(k.SCR_ScorpioSelected) xErrLimit = k.CAM_SCR_XErrLimit;
            else xErrLimit = k.CAM_GTH_XErrLimit;

            //5 different vision targets: loading station, rocket hatch, rocket cargo, cargoship hatch, cargoship cargo
            if(in.cargoNotHatch){
                //if in cargo mode
                if(in.controlBoard.nearFarCargo == NearFarCargo.CARGO){
                    //cargo ship
                    if(k.SCR_ScorpioSelected) {
                        vOffset = k.CAM_SCR_Cargo_CS_Offset;
                        xErrLimit = k.CAM_SCR_Cargo_CS_XErrLimit;
                    } else {
                        vOffset = k.CAM_GTH_Cargo_CS_Offset;
                        xErrLimit = k.CAM_GTH_Cargo_CS_XErrLimit;
                    }
                } else {
                    //rocket
                    if(k.SCR_ScorpioSelected) {
                        vOffset = k.CAM_SCR_Cargo_RKT_Offset;
                        xErrLimit = k.CAM_SCR_Cargo_RKT_XErrLimit;
                    } else {
                        vOffset = k.CAM_GTH_Cargo_RKT_Offset;
                        xErrLimit = k.CAM_GTH_Cargo_RKT_XErrLimit;
                    }
                }
            } else {
                //hatch mode
                if(sense.hasHatch){
                    if(in.controlBoard.nearFarCargo == NearFarCargo.CARGO){
                        //cargo ship
                        if(k.SCR_ScorpioSelected) vOffset = k.CAM_SCR_Hatch_CS_Offset;
                        else                      vOffset = k.CAM_GTH_Hatch_CS_Offset;
                    } else {
                        //rocket
                        if(k.SCR_ScorpioSelected) vOffset = k.CAM_SCR_Hatch_RKT_Offset;
                        else                      vOffset = k.CAM_GTH_Hatch_RKT_Offset;
                    }
                } else {
                    //loading station
                    if(k.SCR_ScorpioSelected) vOffset = k.CAM_SCR_Hatch_LS_Offset;
                    else                      vOffset = k.CAM_GTH_Hatch_LS_Offset;
                }
            }
            if(k.SCR_ScorpioSelected){
                vOffset += k.CAM_SCR_MainTargetOffset; //main offset to all positions
            } else {
                vOffset += k.CAM_GTH_MainTargetOffset; //main offset to all positions
            }            

            double vDist = vd.distance-vOffset;
            double vXErr = (vd.distance + 15) * Math.tan(Angle.toRad(vd.angleTo));
            
            //correct for RSE distance covered
            double dRSEx = rse.x - vd.rseX;
            double dRSEy = rse.y - vd.rseY;
            //rotate by the robot angle to get the robot relative dx an dy
            double robotAngleRad = -sense.robotAngle.getRad();
            double dRobotX = dRSEx * Math.cos(robotAngleRad) - dRSEy * Math.sin(robotAngleRad);
            double dRobotY = dRSEx * Math.sin(robotAngleRad) + dRSEy * Math.cos(robotAngleRad);
            //subtract diff value to compensate for movement between now and last image
            vDist -= dRobotY;
            vXErr -= dRobotX;

            SmartDashboard.putBoolean("Stage2", true);

            //if the angle indicates that we should be in stage 1
            boolean disableAutoShoot = Math.abs(elevator.getElevatorError()) > 2; //disable auto shoot while in stage 1 or elevator not ready
            if(Math.abs(vXErr) > xErrLimit || disableAutoShoot) {
                //we should be in stage 1, so recalc distances
                if(k.SCR_ScorpioSelected) vDist -= k.CAM_SCR_Stage1Offset;
                else vDist -= k.CAM_GTH_Stage1Offset;
                disableAutoShoot = true;
                SmartDashboard.putBoolean("Stage2", false);
            }
            
            //Calculate actual angle to (From front of bot, not camera)
            double vAngle = Math.atan2(vXErr,vDist);

            if(k.SCR_ScorpioSelected){
                vAngle *= 1;
            } else {
                vAngle *= 1;//1.25;
            }
            
            
            //PID to distance amplitude of vector
            double vHypot = Math.sqrt(vXErr*vXErr + vDist*vDist);//Math.abs((vDist)/Math.cos(vAngle));
            double velMag = Math.sqrt(rse.dx*rse.dx + rse.dy*rse.dy) / sense.dt;
            double filtVel = lpf.run(velMag);
            double vPwr = vHypot * k.DRV_TargetDistanceKP + filtVel * k.DRV_TargetDistanceKD;
            if(Math.abs(vPwr) < k.DRV_CamDriveMinPwr_X){
                vPwr += Math.signum(vPwr) * k.DRV_CamDriveMinPwr_X;
            }
            double vAmplitude = Util.limit(vPwr, k.DRV_CamDriveMaxPwr_Y);// - k.DRV_CamDriveMinPwr_Y;
            
            SmartDashboard.putNumber("vDist", vDist);
            SmartDashboard.putNumber("vXErr", vXErr);
            SmartDashboard.putNumber("vAngle", vAngle);
            SmartDashboard.putNumber("vAmplitude", vAmplitude);


            //Break vector into X and Y components
            double vX = vAmplitude * Math.sin(vAngle);
            double vY = vAmplitude * Math.cos(vAngle);
            //Send to drivetrain
            double rotPower = pidOrient();
            
            SmartDashboard.putNumber("vHypot",vHypot);


            double allowableXErr;
            if(in.controlBoard.nearFarCargo == NearFarCargo.CARGO && in.cargoNotHatch){
                if(k.SCR_ScorpioSelected) allowableXErr = k.CAM_SCR_AllowShootCargoCSXErr;
                else allowableXErr = k.CAM_GTH_AllowShootCargoCSXErr;
            } else {
                if(in.cargoNotHatch){
                    if(k.SCR_ScorpioSelected) allowableXErr = k.CAM_SCR_AllowShootCargoRKTXErr;
                    else allowableXErr = k.CAM_GTH_AllowShootCargoRKTXErr;
                } else {
                    if(k.SCR_ScorpioSelected) allowableXErr = k.CAM_SCR_AllowShootXErr;
                    else allowableXErr = k.CAM_GTH_AllowShootXErr;
                }
            }
            
            //if picking up hatch with mk1
            if(!in.cargoNotHatch && !sense.hasHatch && !k.SCR_ScorpioSelected){ 
                autoShoot = true;
            } else {//all else
                autoShoot = !disableAutoShoot && vDist < k.CAM_ShootDist && Math.abs(vXErr) < allowableXErr;
            }
            // no drive forward into cargo ship
			if(autoShoot && sense.hasHatch && in.controlBoard.nearFarCargo != NearFarCargo.CARGO && !k.SCR_ScorpioSelected){ //override driving to compress bumpers
				swerve(0, -k.DRV_CamHatchDeliverForwardPower, 0);
            } else if(gatherer.scorpioActive()){
                swerve(0,0,0);
            } else {
                swerve(vX, vY, rotPower);
            }
            SmartDashboard.putNumber("camDrive_AllowableAngle",allowableXErr);
            SmartDashboard.putBoolean("disableAutoShoot",disableAutoShoot);
            SmartDashboard.putNumber("vOffset",vOffset);
            
            
        }
    }

}