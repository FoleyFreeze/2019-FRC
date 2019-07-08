package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.ControlBoard.NearFarCargo;
import frc.robot.subsystems.autodrive.Point;
import frc.robot.util.Angle;
import frc.robot.util.Util;

public class RSE extends Component {

    public double x;
    public double y;
    public double dx;
    public double dy;
    public double theta;

    private double[] prevEnc;
    private double[] prevWheelAngle;
    private double prevRobotAngle;

    public SendableChooser<Integer> startSelector;

    public RSE() {
        prevEnc = new double[4];
        prevWheelAngle =  new double[4]; 

        startSelector = new SendableChooser<>();
        startSelector.setDefaultOption("Far Left", 0);
        startSelector.addOption("Mid Left", 1);
        startSelector.addOption("Mid Right", 2);
        startSelector.addOption("Far Right", 3);
        //SmartDashboard.putData("StartLocation",startSelector);
        
        timestamps = new double[20]; //20ms loop * 20 captures the past 400ms
        rseXs = new double[20];
        rseYs = new double[20];

		reset();
    }

    int selection;

    public void reset() {
        //int selection = startSelector.getSelected();
        y = k.AD_RobotHeight/2 + k.AD_HabY;
        
        switch(selection){
            case 0:
                x = -k.AD_HabEdgeX + k.AD_RobotWidth/2;
            break;
            case 1:
                x = -k.AD_MidEdgeX + k.AD_RobotWidth/2;
            break;
            case 2:
                x = k.AD_MidEdgeX - k.AD_RobotWidth/2;
            break;
            case 3:
                x = k.AD_HabEdgeX - k.AD_RobotWidth/2;
            break;
            default:
                x = 0;
                y = 0;
        }

        //x = -k.AD_HabEdgeX + k.AD_RobotWidth/2;
        //y = k.AD_HabY + k.AD_RobotHeight/2;
        theta = sense.robotAngle.getDeg();
        prevRobotAngle = theta;

        for(int i = 0; i< prevEnc.length;++i) {
            prevEnc[i] = sense.driveEnc[i];
            prevWheelAngle[i] = sense.angles[i].getDeg(); 
        } 

    }

    public String habStartLoc;

    public void run() {

        //run settings for re-zero
        if(in.leftNotRight){
            //if(in.controlBoard.nearFarCargo == NearFarCargo.CARGO && in.controlBoard.rocketCargoState == RocketCargoshipPosition.FRONT){
            //    habStartLoc = "MID LEFT";
            //    selection = 1;
            //} else {
                habStartLoc = "FAR LEFT";
                selection = 0;
            //}
        } else {
            //if(in.controlBoard.nearFarCargo == NearFarCargo.CARGO && in.controlBoard.rocketCargoState == RocketCargoshipPosition.FRONT){
            //    habStartLoc = "MID RIGHT";
            //    selection = 2;
            //} else {
                habStartLoc = "FAR RIGHT";
                selection = 3;
            //}
        }
        SmartDashboard.putString("HabStartLoc",habStartLoc);

        //determine when to re-zero (only when picking up hatches and facing a loading station)
        if(sense.hasHatchEdge && sense.hasHatch && !in.shift && Math.abs(sense.robotAngle.subDeg(180)) < 10) {
            if(in.autoDrive){ //if in autoDrive, use left/right switch to know which loading station we are at
                if(in.leftNotRight){
                    SmartDashboard.putNumber("RSE_ErrX",x - -k.AD_LoadingStationX);
                    SmartDashboard.putNumber("RSE_ErrY",y - k.AD_RobotHeight/2);
                    x = -k.AD_LoadingStationX;
                    y = k.AD_RobotHeight/2;
                } else {
                    SmartDashboard.putNumber("RSE_ErrX",x - k.AD_LoadingStationX);
                    SmartDashboard.putNumber("RSE_ErrY",y - k.AD_RobotHeight/2);
                    x = k.AD_LoadingStationX;
                    y = k.AD_RobotHeight/2;
                }
            } else { //if not in autoDrive, attempt to guess based on present RSE location
                double dist1 = Util.dist(-k.AD_LoadingStationX,k.AD_RobotHeight/2, x,y);
                double dist2 = Util.dist(k.AD_LoadingStationX,k.AD_RobotHeight/2, x,y);
                if(dist1 < dist2){
                    SmartDashboard.putNumber("RSE_ErrX",x - -k.AD_LoadingStationX);
                    SmartDashboard.putNumber("RSE_ErrY",y - k.AD_RobotHeight/2);
                    x = -k.AD_LoadingStationX;
                    y = k.AD_RobotHeight/2;
                } else {
                    SmartDashboard.putNumber("RSE_ErrX",x - k.AD_LoadingStationX);
                    SmartDashboard.putNumber("RSE_ErrY",y - k.AD_RobotHeight/2);
                    x = k.AD_LoadingStationX;
                    y = k.AD_RobotHeight/2;
                }
            }

            //reset all the things
            prevRobotAngle = theta;
            for(int i = 0; i< prevEnc.length;++i) {
                prevEnc[i] = sense.driveEnc[i];
                prevWheelAngle[i] = sense.angles[i].getDeg(); 
            } 

        //reset on rocket hatch deliveries
        } else if(sense.hasHatchEdge && !sense.hasHatch && !in.shift && in.controlBoard.nearFarCargo != NearFarCargo.CARGO){
            double xSign = 1;
            if(in.leftNotRight) xSign = -1;
            if(in.controlBoard.nearFarCargo == NearFarCargo.NEAR){
                double xDest = k.AD_RocketXHatch * xSign;
                double yDest = k.AD_NearRocketYHatch;
                double distError = Util.dist(x, y, xDest, yDest);
                if(distError < k.AD_MaxDistError){
                    SmartDashboard.putNumber("RSE_ErrX",x - xDest);
                    SmartDashboard.putNumber("RSE_ErrY",y - yDest);
                    x = xDest;
                    y = yDest;
                }
            }else{
                double xDest = k.AD_RocketXHatch * xSign;
                double yDest = k.AD_FarRocketYHatch;
                double distError = Util.dist(x, y, xDest, yDest);
                if(distError < k.AD_MaxDistError){
                    SmartDashboard.putNumber("RSE_ErrX",x - xDest);
                    SmartDashboard.putNumber("RSE_ErrY",y - yDest);
                    x = xDest;
                    y = yDest;
                }
            }
        
        //reset on rocket cargo deliveries
        } else if(sense.hasCargoEdge && !sense.hasCargo && !in.shift){
            double xSign = 1;
            if(in.leftNotRight) xSign = -1;
            double xDest = k.AD_RocketXCargo * xSign;
            double yDest = k.AD_RocketYCargo;
            double distError = Util.dist(x, y, xDest, yDest);
            if(distError < k.AD_MaxDistError){
                SmartDashboard.putNumber("RSE_ErrX",x - xDest);
                SmartDashboard.putNumber("RSE_ErrY",y - yDest);
                x = xDest;
                y = yDest;
            }


        //reset on falling off hab2
        } else if(in.autoDrive && autoDriving.startingHab2 && Math.abs(sense.zAccel) > k.AD_Hab2ResetAccel){
            autoDriving.startingHab2 = false;
            if(in.leftNotRight){
                x = -k.AD_HabEdgeX + k.AD_RobotWidth/2; 
                y = k.AD_HabY + k.AD_RobotHeight/2 + k.AD_Hab2_RSE_Yoffset;
            } else {
                x = k.AD_HabEdgeX - k.AD_RobotWidth/2; 
                y = k.AD_HabY + k.AD_RobotHeight/2 + k.AD_Hab2_RSE_Yoffset;
            }
        }



        // read navx
        double deltaRobotAngle = sense.robotAngle.subDeg(prevRobotAngle);
        prevRobotAngle = sense.robotAngle.getDeg();
        SmartDashboard.putNumber("RSEdTheta", deltaRobotAngle);
        double sumDX = 0;
        double sumDY = 0;

        for(int i = 0; i < 4; i++) {
            // average of wheel angles
            double deltaWheelAng = sense.angles[i].subDeg(prevWheelAngle[i]);
            double avgWheelAng = deltaWheelAng*.5 + prevWheelAngle[i];
            prevWheelAngle[i] = sense.angles[i].getDeg();
            SmartDashboard.putNumber("RSEwheelAngle"+i,avgWheelAng);
            
            // delta of wheel encoders
            double deltaDriveEnc = sense.driveEnc[i] - prevEnc[i];
            prevEnc[i] = sense.driveEnc[i];
            SmartDashboard.putNumber("RSEdDriveEnc"+i, deltaDriveEnc);

            //wheel r, theta
            double r = deltaDriveEnc;
            double theta = avgWheelAng + deltaWheelAng/2;
            SmartDashboard.putNumber("RSEWheeltheta"+i, theta);

                // back to radians
            theta *= Math.PI/180;

            double deltaX = r * Math.cos(theta);
            double deltaY = r * Math.sin(theta);
            
            // sum the field delta x and delta y for average
            sumDX += deltaX;
            sumDY += deltaY;
            
        }
         
        // divide total by 4 for calculating average
        sumDX /= 4;
        sumDY /= 4;

        //convert to field dx 
        //TODO: consider using prevAngle + half the deltaAngle
        //using prev - delta because at this point, prev contains the current angle
        double radAngle = Angle.toRad(prevRobotAngle - deltaRobotAngle*0.5);
        dx = sumDX * Math.cos(radAngle) - sumDY * Math.sin(radAngle);
        dy = sumDX * Math.sin(radAngle) + sumDY * Math.cos(radAngle);

        // update stored x and y 
        if(autoDriving.startingHab2){
            //dont update x and y when on hab2
        } else {
            x += dx;
            y += dy;
        }


        //stay in the field
        if(x > k.AD_FieldMaxX) x = k.AD_FieldMaxX;
        else if(x < -k.AD_FieldMaxX) x = -k.AD_FieldMaxX;
        if(y > k.AD_FieldMaxY) y = k.AD_FieldMaxY;
        else if(y < 0) y = 0;

        //update the rse history
        rseXs[idxOffset] = x;
        rseYs[idxOffset] = y;
        timestamps[idxOffset] = Timer.getFPGATimestamp();
        idxOffset++;
        if(idxOffset >= rseXs.length) idxOffset = 0;

        SmartDashboard.putNumber("RSE dX", sumDX);
        SmartDashboard.putNumber("RSE dY", sumDY);
        SmartDashboard.putNumber("RSE X", x);
        SmartDashboard.putNumber("RSE Y", y);
    }

    private double[] timestamps;
    private double[] rseXs;
    private double[] rseYs;
    private int idxOffset = 0;
    private Point retPoint = new Point();
    public Point getPositionAtTime(double t){

        int length = timestamps.length;
        int end = idxOffset - 1;
        if(end < 0) end += length;

        if(t <= timestamps[idxOffset]) {
            retPoint.x = rseXs[idxOffset];
            retPoint.y = rseYs[idxOffset];
        } else if(t >= timestamps[end]) {
            retPoint.x = rseXs[end];
            retPoint.y = rseYs[end];
        } else {
            int i=0;
            for(;i<length;i++){
                if(t < timestamps[(i+idxOffset) % length]){
                    break;
                }
            }

            int idxMinus1 = (i-1+idxOffset) % length;
            int idx = (i+idxOffset) % length;

            double frac = (t - timestamps[idxMinus1])/(timestamps[idx]-timestamps[idxMinus1]);
            retPoint.x = frac*(rseXs[idx] - rseXs[idxMinus1]) + rseXs[idxMinus1];
            retPoint.y = frac*(rseYs[idx] - rseYs[idxMinus1]) + rseYs[idxMinus1];
        }
        
        return retPoint;
    }

}

