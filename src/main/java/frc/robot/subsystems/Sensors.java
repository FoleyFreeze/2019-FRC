package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.io.K;

public class Sensors extends Component {
    private AHRS navx;
    private AnalogInput[] angleEnc = new AnalogInput[4];

    public double robotAngle;
    public double[] angles = new double[4];

    public Sensors() {
        angleEnc[0] = new AnalogInput(0);
        angleEnc[1] = new AnalogInput(1);
        angleEnc[2] = new AnalogInput(2);
        angleEnc[3] = new AnalogInput(3);
        navx = new AHRS(Port.kMXP);
    }

    public void init() {
        navx.zeroYaw();
    }

    public void run() {
       double angleFL = angleEnc[0].getAverageVoltage()/5.0*360.0 - K.SEN_AbsAngleFL;
       double angleFR = angleEnc[1].getAverageVoltage()/5.0*360.0 - K.SEN_AbsAngleFR;
       double angleRL = angleEnc[2].getAverageVoltage()/5.0*360.0 - K.SEN_AbsAngleRL;
       double angleRR = angleEnc[3].getAverageVoltage()/5.0*360.0 - K.SEN_AbsAngleRR;
       angles[0] = angleFL;
       angles[1] = angleFR;
       angles[2] = angleRL;
       angles[3] = angleRR;

       robotAngle = navx.getYaw();
    }
}