package frc.robot.subsystems.autodrive;

import java.util.Stack;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Component;
import frc.robot.util.Util;
import frc.robot.util.Filter;

public class AutoDrive extends Component{
    
    Stack<Node> path;
    public Point targetPoint;
    public boolean pathComplete;
    public boolean enableAutoTurn;

    private int edgeStatus;
    private int startingPolyId;

    public int currPolyId;
    public double lastDist;

    public AutoDrive(){
        targetPoint = null;
        pathComplete = false;
    }

    public void run(){
        if(targetPoint != null){
            SmartDashboard.putNumber("TargetX", targetPoint.x);
            SmartDashboard.putNumber("TargetY", targetPoint.y);
        } else {
            SmartDashboard.putNumber("TargetX", 0);
            SmartDashboard.putNumber("TargetY", 0);
        }
        SmartDashboard.putBoolean("pathComplete", pathComplete);

        if(k.AD_Disabled || !in.autoDrive) {
            pathComplete = false;
            return;
        }
        
        //if first time, create a new path
        if(in.autoDriveRising || sense.hasHatchEdge) {
            path = pathfinder.determinePath();
            powerLim = 0;//reset the power limit so we smoothly accel
            if(path != null && !path.isEmpty()){
                //get rid of the first node, because it is not a target, it is our starting position
                startingPolyId = path.peek().poly.id;

                path.pop();
                //if there is more than 1 step in the path, disable turning for the first step
                enableAutoTurn = path.size() <= 1;
                for(Node n : path){
                    System.out.println("Poly: " + n.poly.id + " Edge: " + n.location.x + "," + n.location.y);
                }
                if(!path.isEmpty()){
                    Node n = path.peek();
                    edgeStatus = getEdgeCrossing(n.location, n.edgePoint,rse.x,rse.y);
                }
            } else {
                System.out.println("Null path");
            }
        }

        if(path == null) {
            targetPoint = null;
            pathComplete = false;
            return; //cant follow path if there is no path
        }

        //also complete the path when we have seen 3 good vision targets
        if(path.isEmpty() || path.size() == 1 && view.lastTargetsGood(3)) {
            //indicate that the path is complete
            pathComplete = true;
            targetPoint = null;
            return;
        }
        Node n = path.peek();
        currPolyId = n.poly.id;
        //until we cross the edge, PID to its center point
        //or if we get within 6in of target
        boolean isClose = Util.dist(new Point(rse.x,rse.y), n.location) < 6;

        //edge check is true if we have not yet broken the plane to leave the current poly
        boolean edgeCheck = edgeStatus == getEdgeCrossing(n.location,n.edgePoint,rse.x,rse.y);
        edgeCheck = edgeCheck || path.size() == 1;

        if(!isClose && edgeCheck){
            targetPoint = n.location;
        } else {//else go to the next polygon
            path.pop();
            enableAutoTurn = true; //once we start the next step, allow auto turn again
            if(!path.isEmpty()){
                n = path.peek();
                edgeStatus = getEdgeCrossing(n.location,n.edgePoint,rse.x,rse.y);
                targetPoint = n.location;
            }
        }

    }

    double powerLim;
    Point retPoint = new Point();
    private Filter lpf = new Filter(0.5, true, 0.06, 0);
    public Point getDrivePower(){

        if(path.isEmpty()) {
            retPoint.x = 0;
            retPoint.y = 0;
            return retPoint;
        }

        Node n = path.peek();
        int thisPoly = n.poly.id;

        double endPowerLim;
        if(thisPoly == 0 || thisPoly == 18){
            endPowerLim = k.AD_MaxPowerHab;
        } else {
            endPowerLim = k.AD_MaxPower;
        }
        
        if(powerLim < endPowerLim){
            powerLim += k.AD_AccelLim * sense.dt;
            powerLim = Math.min(powerLim, endPowerLim);
        }

        //calc powers for X and Y based on target point and rse
        double distX = targetPoint.x - rse.x;
        double distY = targetPoint.y - rse.y;


        //limit power increase per timestep

        //PID and limit magnitude
        double r = Math.sqrt(distX*distX + distY*distY);
        double blendLim; 
        if (path.size() >= 2) blendLim = Math.min(powerLim*r/k.AD_BlendDist, powerLim);
        else blendLim = powerLim;
        double velMag = Math.sqrt(rse.dx*rse.dx + rse.dy*rse.dy) / sense.dt;
        double filtVel = lpf.run(velMag);
        double rPwr = Util.limit(r * k.AD_AutoDriveKP + filtVel * k.AD_AutoDriveKD, blendLim);//powerLim);//blendlim
        double theta = Math.atan2(distY,distX);
        lastDist = r;
        double autoX = rPwr * Math.cos(theta);
        double autoY = rPwr * Math.sin(theta);

        //if power is low, get the next point and add its value
        
        if(rPwr < powerLim && path.size() >= 2){
            double blendPwr = powerLim - Math.abs(rPwr);
            
            Node n2 = path.get(path.size() - 2); //get the second thing off the stack

            double distX2 = n2.edgePoint.x - rse.x;
            double distY2 = n2.edgePoint.y - rse.y;
            double r2 = Math.sqrt(distX2*distX2 + distY2*distY2);
            double rPwr2 = Util.limit(r2 * k.AD_AutoDriveKP, blendPwr);

            double xPwr2 = rPwr2 * Math.cos(theta);
            double yPwr2 = rPwr2 * Math.sin(theta);

            //add them together
            autoX += xPwr2;
            autoY += yPwr2;
        }

        retPoint.x = autoX;
        retPoint.y = autoY;

        return retPoint;
    }

    public int getEdgeCrossing(Point p1, Point p2, double x, double y){
        //does a ray cast from x,y intersect the line from p1 to p2
        boolean xCross;
        boolean yCross;

        if(p1.y == p2.y){
            //this is the only time we don't intersect
            //return y == p1.y;
            xCross = false;
        } else {
            //we need to interp what the edge's x will be at our y value, then it intersects if x < vert(x)
            double frac = (y - p2.y)/(p1.y - p2.y);
            double edgeX = frac*(p1.x - p2.x) + p2.x;

            //return x < edgeX;
            xCross = x < edgeX;
        }
        
        //check y crossing
        if(p1.x == p2.x){
            //this is the only time we don't intersect
            //return y == p1.y;
            yCross = false;
        } else {
            //we need to interp what the edge's x will be at our y value, then it intersects if x < vert(x)
            double frac = (x - p2.x)/(p1.x - p2.x);
            double edgeY = frac*(p1.y - p2.y) + p2.y;

            //return x < edgeX;
            yCross = y < edgeY;
        }

        if(!xCross && !yCross) return 0;
        else if(xCross && !yCross) return 1;
        else if(!xCross && yCross) return 2;
        else return 3;
    }

}