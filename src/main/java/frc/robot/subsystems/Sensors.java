package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.ElectroJendz;
import frc.robot.util.Angle;

public class Sensors extends Component {
    private AHRS navx;
    private AnalogInput[] angleEnc = new AnalogInput[4];
    public PowerDistributionPanel pdp;

    public Angle robotAngle = new Angle();
    public double deltaRobotAngle = 0;
    private double prevRobotAngle = 0;
    public Angle[] angles = new Angle[4];
    public double elevatorEncoder;
    public double[] driveEnc = new double[4];
    public double climberEncoder;
    public double scorpioArmEnc;

    public boolean hasCargo;
    public boolean prevHasCargo;
    public boolean hasCargoEdge;
    public boolean hasHatch;
    public boolean hasHatchEdge;
    public boolean prevHasHatch;
    public boolean isDisabled;
    public boolean isAuto;
    public boolean isTeleop;
    public boolean navxGood;

    public double dt;
    private double prevTime;

    public Sensors() {
        angleEnc[0] = new AnalogInput(ElectroJendz.FL_ENC_ID);
        angleEnc[1] = new AnalogInput(ElectroJendz.FR_ENC_ID);
        angleEnc[2] = new AnalogInput(ElectroJendz.RL_ENC_ID);
        angleEnc[3] = new AnalogInput(ElectroJendz.RR_ENC_ID);
        navx = new AHRS(Port.kMXP);
        pdp = new PowerDistributionPanel();

        //init angle objects
        for(int i=0; i<angles.length; i++) {
            angles[i] = new Angle();
        }
    }

    public void init() {
        navx.zeroYaw();//reset navx
        robotAngle.setDeg(k.SEN_NAVX_Offset);
        out.resetEnc();
        rse.reset(); 

        nxX = 0;
        nxY = 0;
        nxOffsetX = 0;
        nxOffsetY = 0;
        navx.resetDisplacement();
    }

    double[] rawAngles = {0,0,0,0};
    public void run() {
        double angleFL = angleEnc[0].getAverageVoltage()/5.0*360.0 - k.SEN_AbsAngleFL;
        double angleFR = angleEnc[1].getAverageVoltage()/5.0*360.0 - k.SEN_AbsAngleFR;
        double angleRL = angleEnc[2].getAverageVoltage()/5.0*360.0 - k.SEN_AbsAngleRL;
        double angleRR = angleEnc[3].getAverageVoltage()/5.0*360.0 - k.SEN_AbsAngleRR;
        angles[0].setDeg(angleFL);
        angles[1].setDeg(angleFR);
        angles[2].setDeg(angleRL);
        angles[3].setDeg(angleRR);
        
        rawAngles[0] = angleFL;
        rawAngles[1] = angleFR;
        rawAngles[2] = angleRL;
        rawAngles[3] = angleRR;
        //SmartDashboard.putNumberArray("WheelAngles", rawAngles);

        navxGood = navx.isConnected();
        SmartDashboard.putBoolean("navxgood", navxGood);
        estNavxPosition();

        robotAngle.setDeg(-navx.getYaw() + k.SEN_NAVX_Offset); 
        deltaRobotAngle = robotAngle.subDeg(prevRobotAngle);
        prevRobotAngle = robotAngle.getDeg();
        SmartDashboard.putNumber("Angle of Robot", robotAngle.getDeg());
        out.getEnc();

        double time = Timer.getFPGATimestamp();
        dt = time - prevTime;
        prevTime = time;
        SmartDashboard.putNumber("dt",dt);

        SmartDashboard.putBoolean("HasCargo",hasCargo);
        SmartDashboard.putBoolean("HasHatch",hasHatch);
    }

    public double nxX = 0;
    public double nxY = 0;
    private double nxOffsetX = 0;
    private double nxOffsetY = 0;
    private void estNavxPosition(){
        //when in park mode (i.e. robot is stopped) reset the integrator
        if(drive.parkMode){
            nxOffsetX = nxX;
            nxOffsetY = nxY;
            navx.resetDisplacement();
        }

        nxX = nxOffsetX + navx.getDisplacementX() * 100/2.54;//convert from meters to inches
        nxY = nxOffsetY + navx.getDisplacementY() * 100/2.54;

        SmartDashboard.putNumber("NavxX", nxX);
        SmartDashboard.putNumber("NavxY", nxY);
    }
}




