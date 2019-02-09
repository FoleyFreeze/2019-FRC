package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.ElectroJendz;
import frc.robot.util.Angle;

public class Sensors extends Component {
    private AHRS navx;
    private AnalogInput[] angleEnc = new AnalogInput[4];

    public Angle robotAngle = new Angle();
    public Angle[] angles = new Angle[4];
    public double elevatorEncoder;
    
    public boolean hasBall;
    public boolean hasHatch;


    public Sensors() {
        angleEnc[0] = new AnalogInput(ElectroJendz.FL_ENC_ID);
        angleEnc[1] = new AnalogInput(ElectroJendz.FR_ENC_ID);
        angleEnc[2] = new AnalogInput(ElectroJendz.RL_ENC_ID);
        angleEnc[3] = new AnalogInput(ElectroJendz.RR_ENC_ID);
        navx = new AHRS(Port.kMXP);

        //init angle objects
        for(int i=0; i<angles.length; i++) {
            angles[i] = new Angle();
        }
    }

    double navXoffset = 90;
    public void init() {
        navx.zeroYaw();//reset navx
    }

    double[] rawAngles = {0,0,0,0};
    public void run() {
        double angleFL = angleEnc[0].getAverageVoltage()/5.0*360.0 - k.SEN_AbsAngleFL;
        double angleFR = angleEnc[1].getAverageVoltage()/5.0*360.0 - k.SEN_AbsAngleFR;
        double angleRL = angleEnc[2].getAverageVoltage()/5.0*360.0 - k.SEN_AbsAngleRL;
        double angleRR = angleEnc[3].getAverageVoltage()/5.0*360.0 - k.SEN_AbsAngleRR;
        angles[0].set(angleFL);
        angles[1].set(angleFR);
        angles[2].set(angleRL);
        angles[3].set(angleRR);
        
        rawAngles[0] = angleFL;
        rawAngles[1] = angleFR;
        rawAngles[2] = angleRL;
        rawAngles[3] = angleRR;
        SmartDashboard.putNumberArray("WheelAngles", rawAngles);

        robotAngle.set(-navx.getYaw() + navXoffset);
    }
}




