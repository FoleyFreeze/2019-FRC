package frc.robot.subsystems.autodrive;

import java.util.Stack;

import frc.robot.subsystems.Component;

public class AutoDrive extends Component{
    
    Stack<Node> path;
    public Point targetPoint;
    public boolean pathComplete;

    public AutoDrive(){
        targetPoint = null;
        pathComplete = false;
    }

    public void run(){
        if(k.AD_Disabled || !in.autoDrive) {
            pathComplete = false;
            return;
        }
        
        //if first time, create a new path
        if(in.autoDriveRising || sense.hasHatchEdge) path = pathfinder.determinePath();

        if(path == null) {
            targetPoint = null;
            pathComplete = false;
            return; //cant follow path if there is no path
        }

        Node n = path.peek();
        if(n == null) {
            //indicate that the path is complete
            pathComplete = true;
            targetPoint = null;
            return;
        }

        //if we are still in the polygon, PID to point
        if(n.poly.containsPoint(rse.x, rse.y)){
            targetPoint = n.location;
        } else {//else go to the next polygon
            path.pop();
        }

    }
}