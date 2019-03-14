package frc.robot.subsystems.autodrive;

public class Polygon{
    int id;
    
    Point[] vert;
    Point[] edges;
    int[] neighbors;

    boolean isDest;
    double angle;

    public Polygon(int id, double[] vertX, double vertY[], int[] edgeID, int[] neighborID){
        this.id = id;

        vert = new Point[vertX.length];
        edges = new Point[edgeID.length];
        neighbors = new int[neighborID.length];

        //copy over the neighbor array
        for(int i=0;i<neighborID.length;i++){
            neighbors[i] = neighborID[i];
        }

        int edgeIdx = 0;
        int nextEdge = edgeID[edgeIdx];

        for(int i = 0; i < vertX.length; i++){
            vert[i] = new Point(vertX[i], vertY[i]);

            //if this is true, make an edge between this and the previous vertex
            if(nextEdge == i) {
                double midX = (vert[i].x + vert[i-1].x) / 2;
                double midY = (vert[i].y + vert[i-1].y) / 2;

                edges[edgeIdx++] = new Point(midX, midY);
                if(edgeIdx < edgeID.length){
                    nextEdge = edgeID[edgeIdx];
                }
            }
        }

    }

    public void setDest(Point edgePoint, double destAngle){
        isDest = true;
        angle = destAngle;

        if(edgePoint != null) edges[0] = edgePoint;
    }
    
    public boolean containsPoint(double x, double y){
        int crossCount = 0;
        Point prevVert = vert[vert.length-1];
        for(int i=0; i<vert.length; i++){
            Point currVert = vert[i];
            if(y >= currVert.y && y <= prevVert.y || y <= currVert.y && y >= prevVert.y){
                if(currVert.x > x || prevVert.x > x){
                    
                }
            }
        }
        return false;
    }
}