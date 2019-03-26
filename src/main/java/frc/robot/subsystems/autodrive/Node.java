package frc.robot.subsystems.autodrive;

import frc.robot.util.Util;

public class Node {
    public Point location;
    public Polygon poly;
    public Node prevNode;
    public Point edgePoint;

    public double traveledDist;
    public double totalDist;

    public Node(Point point, Polygon nextPoly, Node prevNode, Point destPoint, Point edgePoint){
        this.edgePoint = edgePoint;
        location = point;
        poly = nextPoly;
        this.prevNode = prevNode;

        if(prevNode == null){
            traveledDist = 0;
        } else {
            traveledDist = Util.dist(location, prevNode.location) + prevNode.traveledDist;
        }

        totalDist = traveledDist + Util.dist(location, destPoint);
    }

    public String toString(){
        String s = "";

        Node n = this;
        s += n.poly.id + "<-";
        while(n.prevNode != null){
            n = n.prevNode;
            s += n.poly.id + "<-";
        }
        return s;
    }
}