package frc.robot.subsystems.autodrive;

import java.util.Stack;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Component;
import frc.robot.util.Util;

public class AutoDrive extends Component{
    
    Stack<Node> path;
    public Point targetPoint;
    public boolean pathComplete;

    private boolean edgeStatus;

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
            if(path != null){
                //get rid of the first node, because it is not a target, it is our starting position
                path.pop();
                for(Node n : path){
                    System.out.println("Poly: " + n.poly.id + " Edge: " + n.location.x + "," + n.location.y);
                }
                //Node n = path.peek();
                //edgeStatus = n.poly.getEdgeCrossing(n.prevNode.edgeId,rse.x,rse.y);
            } else {
                System.out.println("Null path");
            }
        }

        if(path == null) {
            targetPoint = null;
            pathComplete = false;
            return; //cant follow path if there is no path
        }

        if(path.isEmpty()) {
            //indicate that the path is complete
            pathComplete = true;
            targetPoint = null;
            return;
        }
        Node n = path.peek();

        //until we cross the edge, PID to its center point
        //or if we get within 6in of target
        boolean isClose = Util.dist(new Point(rse.x,rse.y), n.location) < 6;
        if(!isClose /*&& edgeStatus == n.poly.getEdgeCrossing(n.prevNode.edgeId,rse.x,rse.y)*/){
            targetPoint = n.location;
        } else {//else go to the next polygon
            path.pop();
        }

    }
}