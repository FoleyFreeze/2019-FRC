package frc.robot.subsystems.autodrive;

import java.util.PriorityQueue;
import java.util.Stack;
import java.util.Comparator;
import frc.robot.subsystems.Component;

public class Pathfinder extends Component {

    Polygon[] polygonList;
    Polygon[] destPolygons;

    PriorityQueue<Node> pq;

    public Pathfinder(){
        pq = new PriorityQueue<>(new Comparator<Node>(){
            @Override
            public int compare(Node n1, Node n2) {
                return Double.compare(n1.totalDist, n2.totalDist);
            }
        });

        int numPolygons = 17;
        int totalPolygons = numPolygons * 2;

        polygonList = new Polygon[totalPolygons];

        makePoly(new double[]{72.5,  0,    0,   72.5},
                 new double[]{97.5, 97.5, 47.5, 47.5},
                 new int[]{1},
                 false, null, 0);
        
        makePoly(new double[]{ 53.75, 0,  0,   72.5,  53.75},
                 new double[]{175,  175, 97.5, 97.5, 147.5},
                 new int[]{1, 2, 3, 4},
                 false, null, 1);
        
        makePoly(new double[]{ 30,     0,     0,  53.75},
                 new double[]{222.5, 222.5, 175, 175},
                 new int[]{3},
                 true, new Point(12.5, 175), 2);
        
        makePoly(new double[]{ 53.75, 30,    30,    53.75},
                 new double[]{242.5, 242.5, 222.5, 175},
                 new int[]{},
                 false, null, 3);
        
        makePoly(new double[]{53.75, 30, 30, 53.75},
                 new double[]{246.25, 246.25, 242.5, 246.25},
                 new int[]{},
                 true, null, 4);
        
        makePoly(new double[]{53.75, 30, 30, 53.75},
                 new double[]{266.25, 266.25, 246.25, 246.25},
                 new int[]{},
                 true, null, 5);
        
        makePoly(new double[]{53.75, 30, 30, 53.75},
                 new double[]{286.25, 286.25, 266.25, 266.25},
                 new int[]{},
                 true, null, 6);
        
        makePoly(new double[]{155, 53.75, 53.75, 95, 95},
                 new double[]{286.25, 286.25, 242.5, 242.5, 292.5},
                 new int[]{},
                 false, null, 7);
        
        makePoly(new double[]{95, 53.75, 53.75, 95},
                 new double[]{242.5, 242.5, 147.5, 231.25},
                 new int[]{},
                 false, null, 8);
        
        makePoly(new double[]{155, 95, 122.5, 155},
                 new double[]{292.5, 242.5, 148.75, 255},
                 new int[]{},
                 true, null, 9);
        
        makePoly(new double[]{122.5, 95, 95, 122.5},
                 new double[]{148.75, 242.5, 231.25, 207.5},
                 new int[]{},
                 true, null, 10);
        
        makePoly(new double[]{122.5, 95, 155, 155},
                 new double[]{207.5, 231.25, 160, 187.5},
                 new int[]{},
                 true, null, 11);
        
        makePoly(new double[]{95, 53.75, 72.5, 155, 155},
                 new double[]{231.25, 147.5, 97.5, 97.5, 160},
                 new int[]{},
                 false, null, 12);
        
        makePoly(new double[]{107.5, 72.5, 72.5, 107.5},
                 new double[]{97.5, 97.5, 50, 50},
                 new int[]{},
                 false, null, 13);
        
        makePoly(new double[]{155, 107.5, 107.5, 155},
                 new double[]{97.5, 97.5, 50, 50},
                 new int[]{},
                 false, null, 14);
        
        makePoly(new double[]{107.5, 85, 85, 107.5},
                 new double[]{50, 50, 0, 0},
                 new int[]{},
                 false, null, 15);
        
        makePoly(new double[]{155, 107.5, 107.5, 155},
                 new double[]{50, 50, 0, 0},
                 new int[]{},
                 true, null, 16);
    }

    public void makePoly(double[] x, double[] y, int[] edges, boolean dest, Point edgePoint, int id){

    }


    public Stack<Point> determinePath(){
        //determine where we are (initial polygon)

        //determine where we want to go (dest polygon)

        //init nodes into priority queue

        //pop the best node

        //while the best node is not at the destination polygon

            //for each edge in the next polygon

                //make a new node

                //set node point to the edge
                
                //set previous node to best node

                //add the new node to the priority queue
                //with priority being the distance between the current edge and the target edge

            //pop the best node

        //turn best node into a path
        
        //clear the priority queue

        //return the best path
        return null;
    }


}