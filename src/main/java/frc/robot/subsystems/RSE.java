package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RSE extends Component {

    public double x;
    public double y;
    public double theta;

    private double[] prevEnc;
    private double[] prevWheelAngle;
    private double prevRobotAngle;

    public RSE() {
        prevEnc = new double [4];
        prevWheelAngle =  new double [4];        
    }

    public void reset() {
        x = 0;
        y = 0;
        theta = sense.robotAngle.get();
        prevRobotAngle = theta;

        for(int i = 0; i< prevEnc.length;++i) {
            prevEnc[i] = sense.driveEnc[i];
            prevWheelAngle[i] = sense.angles[i].get(); 
        } 

    }


    public void run() {
        // read navx
        double deltaRobotAngle = sense.robotAngle.sub(prevRobotAngle);
        SmartDashboard.putNumber("RSEdTheta", deltaRobotAngle);
        double deltaRobotRad = deltaRobotAngle*Math.PI/180;
        double sumDX = 0;
        double sumDY = 0;

        for(int i = 0; i < 4; i++) {
            // average of wheel encoders
            double avgWheelAng = sense.angles[i].add(prevWheelAngle[i])/2;
            prevWheelAngle[i] = avgWheelAng;
            
            // delta of wheel angle
            double deltaDriveEnc = sense.driveEnc[i] - prevEnc[i];
            prevEnc[i] = deltaDriveEnc;
            SmartDashboard.putNumber("RSEdDriveEnc"+i, deltaDriveEnc);

            // calculate radius values
            double radius = deltaDriveEnc / deltaRobotAngle;
            SmartDashboard.putNumber("RSERadius"+i, radius);

            //use the radius values to calculate the wheel relative delta x and delta y
            double deltaX = radius - radius*Math.cos(deltaRobotRad);
            double deltaY = radius * Math.sin(deltaRobotRad);
            SmartDashboard.putNumber("RSEdX", deltaX);
            SmartDashboard.putNumber("RSEdY", deltaY);

            // rotate delta x and delta y to field x and y 
            double theta = Math.atan2(deltaY, deltaX);
                // to degrees
            theta *= 180/Math.PI;

            double r = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            theta = sense.robotAngle.subtrahend(theta - avgWheelAng);
            SmartDashboard.putNumber("RSEWheeltheta", theta);

                // back to radians
            theta *= Math.PI/180;

            deltaX = radius * Math.cos(theta);
            deltaY = radius * Math.sin(theta);
            
            // sum the field delta x and delta y for average
            sumDX += deltaX;
            sumDY += deltaY;
            
        }
        
        // divide total by 4 for calculating average
        sumDX /= 4;
        sumDY /= 4;
        
        // update stored x and y 
        x += sumDX;
        y += sumDY;

        SmartDashboard.putNumber("RSE X", x);
        SmartDashboard.putNumber("RSE Y", y);
    }

}

