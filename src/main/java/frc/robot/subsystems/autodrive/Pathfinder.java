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

            //pop the best path
        
        //return the best path
        return null;
    }


}