package frc.robot.subsystems.autodrive;

public class Polygon{
    int id;
    
    Point[] vert;
    Point[] edges;
    int[] neighbors;
    int[] edgeLoc;

    boolean isDest;
    double angle;

    public Polygon(int id, double[] vertX, double vertY[], int[] edgeID, int[] neighborID){
        this.id = id;

        vert = new Point[vertX.length];
        edges = new Point[edgeID.length];
        edgeLoc = new int[edgeID.length];
        neighbors = new int[neighborID.length];

        //copy over the neighbor array
        for(int i=0;i<neighborID.length;i++){
            neighbors[i] = neighborID[i];
            edgeLoc[i] = edgeID[i];
        }

        int edgeIdx = 0;
        int nextEdge = edgeID[edgeIdx];

        for(int i = 0; i < vertX.length; i++){
            vert[i] = new Point(vertX[i], vertY[i]);
        }

        for(int i = 0; i < vertX.length; i++){
            //if this is true, make an edge between this and the previous vertex
            if(nextEdge == i) {
                int prevIdx = i-1;
                if(prevIdx < 0) prevIdx += vertX.length;

                double midX = (vert[i].x + vert[prevIdx].x) / 2;
                double midY = (vert[i].y + vert[prevIdx].y) / 2;

                edges[edgeIdx++] = new Point(midX, midY);
                if(edgeIdx < edgeID.length){
                    nextEdge = edgeID[edgeIdx];
                }
            }
        }

    }

    public boolean getEdgeCrossing(int edgeIdx, double x, double y){
        //does a ray cast from x,y intersect the edge at edgeIdx?
        int vertIdx = edgeLoc[edgeIdx];
        int prevVertIdx = vertIdx -1;
        if(prevVertIdx < 0) prevVertIdx += vert.length;

        Point p1 = vert[vertIdx];
        Point p2 = vert[prevVertIdx];

        if(p1.y == p2.y){
            //this is the only time we don't intersect
            return y == p1.y;
        } else {
            //we need to interp what the edge's x will be at our y value, then it intersects if x < vert(x)
            double frac = (y - p2.y)/(p1.y - p2.y);
            double edgeX = frac*(p1.x - p2.x) + p2.x;

            return x < edgeX;
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
                    double frac = (y - prevVert.y) / (currVert.y - prevVert.y);
                    double intersectX = frac * (currVert.x - prevVert.x) + prevVert.x;
                    if(intersectX >= x){
                        crossCount++;
                    } 
                } 
            } 
            prevVert = currVert;
        }
        return crossCount % 2 == 1;
    }
}