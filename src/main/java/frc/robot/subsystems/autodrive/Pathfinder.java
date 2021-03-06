package frc.robot.subsystems.autodrive;

import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Stack;

import frc.robot.subsystems.Component;

public class Pathfinder extends Component {

    Polygon[] polygonList;
    Polygon[] destPolygons;

    PriorityQueue<Node> pq;

    int numPolygons = 18;
    int totalPolygons = numPolygons * 2;

    public Pathfinder(){
        pq = new PriorityQueue<>(new Comparator<Node>(){
            @Override
            public int compare(Node n1, Node n2) {
                return Double.compare(n1.totalDist, n2.totalDist);
            }
        });

        polygonList = new Polygon[totalPolygons];

        makePoly(new double[]{0, 75, 75, 0}, //x vertex values
                 new double[]{48, 48, 95, 95}, //y vertex values
                 new int[]{3},  //2?                       //there is an edge between these verticies
                 new int[]{1},                         //that edge connects to this polygon ID
                 false, null, 0);                      //dest polygon, override edge point, this ID number 
        
        makePoly(new double[]{0, 75, 58, 23, 0},
                 new double[]{95, 95, 190, 190, 190},
                 new int[]{0, 1, 2, 3, 4},
                 new int[]{19, 0, 12, 17, 2},//please check*************************
                 false, null, 1);
        
        makePoly(new double[]{ 0, 23, 23, 0},
                 new double[]{190, 190, 220, 220},
                 new int[]{1},
                 new int[]{1},
                 true, null, 2);
        
        makePoly(new double[]{23, 28, 28, 58, 58},
                 new double[]{220, 246, 256, 256, 190},
                 new int[]{4},
                 new int[]{8},
                 false, null, 3);
        
        makePoly(new double[]{28, 28, 58, 58},
                 new double[]{256, 278, 278, 256},
                 new int[]{3},
                 new int[]{7},
                 true, null, 4);
        
        makePoly(new double[]{28, 28, 58, 58},
                 new double[]{278, 300, 300, 278},
                 new int[]{3},
                 new int[]{7},
                 true, null, 5);
        
        makePoly(new double[]{28, 28, 58, 58},
                 new double[]{300, 322, 322, 300},
                 new int[]{3},
                 new int[]{7},
                 true, null, 6);
        
        makePoly(new double[]{58, 58, 58, 58, 161, 161, 104},
                 new double[]{256, 278, 300, 322, 322, 288, 256},
                 new int[]{1, 2, 3, 6, 7},
                 new int[]{4, 5, 6, 9, 8},
                 false, null, 7);
        
        makePoly(new double[]{58, 58, 104, 104},
                 new double[]{190, 256, 256, 202},
                 new int[]{0, 1, 2, 3},
                 new int[]{12, 3, 7, 10},
                 false, null, 8);
        
        makePoly(new double[]{134, 104, 161, 161},
                 new double[]{239, 256, 288, 254},
                 new int[]{2},
                 new int[]{7},
                 true, null, 9);
        
        makePoly(new double[]{104, 104, 134, 134},
                 new double[]{202, 256, 239, 220},
                 new int[]{1},
                 new int[]{8},
                 true, null, 10);
        
        makePoly(new double[]{104, 134, 161, 161},
                 new double[]{202, 220, 205, 170},
                 new int[]{0},
                 new int[]{12},
                 true, null, 11);
        
        makePoly(new double[]{75, 58, 104, 161, 161, 108},
                 new double[]{95, 190, 202, 170, 95, 95},
                 new int[]{0, 1, 2, 3, 5},
                 new int[]{13, 1, 8, 11, 14},
                 false, null, 12);
        
        makePoly(new double[]{75, 75, 108, 108, 90},
                 new double[]{48, 95, 95, 48, 48},
                 new int[]{2, 3, 4},
                 new int[]{12, 14, 15},
                 false, null, 13);
        
        makePoly(new double[]{108, 108, 161, 161},
                 new double[]{48, 95, 95, 48},
                 new int[]{0, 1, 2},
                 new int[]{16, 13, 12},
                 false, null, 14);
        
        makePoly(new double[]{90, 90, 108, 108},
                 new double[]{0, 48, 48, 0},
                 new int[]{2, 3},
                 new int[]{13, 16},
                 false, null, 15);
        
        makePoly(new double[]{108, 108, 161, 161},
                 new double[]{0, 48, 48, 0},
                 new int[]{1, 2},
                 new int[]{15, 14},
                 true, null, 16);
                 
        makePoly(new double[]{23, 23, 58},
                 new double[]{190, 220, 190},
                 new int[]{0},
                 new int[]{1},
                 false, null, 17);
    }

    public void makePoly(double[] x, double[] y, int[] edges, int[] neighbors, boolean dest, Point edgePoint, int id){
        polygonList[id] = new Polygon(id, x, y, edges, neighbors);
        for(int i=0;i<x.length;i++){
            x[i] = -x[i];
        }
        for(int i=0;i<neighbors.length;i++){
            neighbors[i] = (neighbors[i] + numPolygons) % totalPolygons;
        }
        polygonList[id+numPolygons] = new Polygon(id+numPolygons, x, y, edges, neighbors);
    }


    public Stack<Node> determinePath(){
        //determine where we are (initial polygon)
        int startPolyIdx = -1; 
        for(Polygon p : polygonList){
            if (p.containsPoint(rse.x, rse.y)){
                startPolyIdx = p.id;
                break;
            }
        }
        if(startPolyIdx == -1) {
            System.out.println("No start Polygon");
            return null;
        }
        Polygon startPoly = polygonList[startPolyIdx];
        System.out.println("StartPoly: " + startPolyIdx);

        //determine where we want to go (dest polygon)
        int destPolyIdx = getDestPoly();
        if(destPolyIdx == -1) {
            System.out.println("No dest Polygon");
            return null;
        }
        Polygon destPoly = polygonList[destPolyIdx];
        System.out.println("DestPoly: " + destPolyIdx);

        //init nodes into priority queue
        Node bestNode = new Node(new Point(rse.x,rse.y), startPoly, null, destPoly.edges[0], null);

        //pop the best node

        //while the best node is not at the destination polygon
        int aStarCount = 0;
        while(bestNode.poly.id != destPoly.id){
            aStarCount++;
            //for each edge in the next polygon
            for(int i=0; i<bestNode.poly.edges.length; i++){
                Point edge = bestNode.poly.edges[i];
                
                //if(edge.x != bestNode.location.x || edge.y != bestNode.location.y){
                    //can check poly ids instead of edges/points
                if(bestNode.prevNode == null || bestNode.poly.neighbors[i] != bestNode.prevNode.poly.id){
                    //make a new node
                    Node n = new Node(edge, polygonList[bestNode.poly.neighbors[i]], bestNode, destPoly.edges[0], bestNode.poly.vert[bestNode.poly.edgeLoc[i]]);
                    //set node point to the edge
                    //set previous node to best node
                    System.out.println(n.toString() + " travD: " + n.traveledDist + " totalD: " + n.totalDist);

                    //add the new node to the priority queue
                    pq.add(n);
                    //with priority being the distance between the current edge and the target edge
                }
            }

            //pop the best node
            bestNode = pq.poll();

            if(aStarCount > 20) {
                System.out.println("Astar took too long");
                return null;
            }
        }
            
        //turn best node into a path
        Stack<Node> path = new Stack<>();
        path.add(bestNode);
        while(bestNode.prevNode != null){
            bestNode = bestNode.prevNode;
            path.add(bestNode);
        }
        
        //clear the priority queue
        pq.clear();

        //return the best path
        return path;
    }

    private int getDestPoly(){
        int value = getDestPolyHelper();
        if(in.leftNotRight && value != -1) value += numPolygons;

        return value;
    }

    private int getDestPolyHelper(){
        if(!sense.hasHatch && !in.cargoNotHatch){
            return 16;
        }
        if(!sense.hasCargo && in.cargoNotHatch){
            return -1;//auto drive cannot help find cargo
        }

        switch(in.controlBoard.nearFarCargo){
            case CARGO:
                switch(in.controlBoard.rocketCargoState){
                    case HI:
                        return 6;
                    case MID:
                        return 5;
                    case LO:
                        return 4;
                    case FRONT:
                        return 2;
                    case DEFAULT:
                        return -1;
                }

            case NEAR:
                switch(in.controlBoard.rocketCargoState){
                    case HI:
                    case MID:
                    case LO:
                        if(in.cargoNotHatch) return 10;
                        else return 11;

                    case FRONT:
                        return -1;
                    case DEFAULT:
                        return -1;
                }

            case FAR:
                switch(in.controlBoard.rocketCargoState){
                    case HI:
                    case MID:
                    case LO:
                        if(in.cargoNotHatch) return 10;
                        else return 9;

                    case FRONT:
                        return -1;
                    case DEFAULT:
                        return -1;
                }

            case DEFAULT:
                return -1;
                
        }
        return -1;
    }


}