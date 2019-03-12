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
        neighbors = neighborID;

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
    
}