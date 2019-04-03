package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RSE extends Component {

    public double x;
    public double y;
    public double dx;
    public double dy;
    public double theta;

    private double[] prevEnc;
    private double[] prevWheelAngle;
    private double prevRobotAngle;

    public RSE() {
        prevEnc = new double[4];
        prevWheelAngle =  new double[4]; 
        reset();       
    }

    public void reset() {
        x = -k.AD_HabEdgeX + k.AD_RobotWidth/2;
        y = k.AD_HabY + k.AD_RobotHeight/2;
        theta = sense.robotAngle.getDeg();
        prevRobotAngle = theta;

        for(int i = 0; i< prevEnc.length;++i) {
            prevEnc[i] = sense.driveEnc[i];
            prevWheelAngle[i] = sense.angles[i].getDeg(); 
        } 

    }


    public void run() {
        // read navx
        double deltaRobotAngle = sense.robotAngle.subDeg(prevRobotAngle);
        prevRobotAngle = sense.robotAngle.getDeg();
        SmartDashboard.putNumber("RSEdTheta", deltaRobotAngle);
        double sumDX = 0;
        double sumDY = 0;

        for(int i = 0; i < 4; i++) {
            // average of wheel angles
            double deltaWheelAng = sense.angles[i].subDeg(prevWheelAngle[i]);
            double avgWheelAng = deltaWheelAng*.5 + prevWheelAngle[i];
            prevWheelAngle[i] = sense.angles[i].getDeg();
            SmartDashboard.putNumber("RSEwheelAngle"+i,avgWheelAng);
            
            // delta of wheel encoders
            double deltaDriveEnc = sense.driveEnc[i] - prevEnc[i];
            prevEnc[i] = sense.driveEnc[i];
            SmartDashboard.putNumber("RSEdDriveEnc"+i, deltaDriveEnc);

            //wheel r, theta
            double r = deltaDriveEnc;
            double theta = avgWheelAng + deltaWheelAng/2;
            SmartDashboard.putNumber("RSEWheeltheta"+i, theta);

                // back to radians
            theta *= Math.PI/180;

            double deltaX = r * Math.cos(theta);
            double deltaY = r * Math.sin(theta);
            
            // sum the field delta x and delta y for average
            sumDX += deltaX;
            sumDY += deltaY;
            
        }
         
        // divide total by 4 for calculating average
        sumDX /= 4;
        sumDY /= 4;

        //convert to field dx 
        double radAngle = prevRobotAngle * Math.PI / 180;
        dx = sumDX * Math.cos(radAngle) - sumDY * Math.sin(radAngle);
        dy = sumDX * Math.sin(radAngle) + sumDY * Math.cos(radAngle);

        // update stored x and y 
        x += dx;
        y += dy;

        SmartDashboard.putNumber("RSE dX", sumDX);
        SmartDashboard.putNumber("RSE dY", sumDY);
        SmartDashboard.putNumber("RSE X", x);
        SmartDashboard.putNumber("RSE Y", y);
    }

}

