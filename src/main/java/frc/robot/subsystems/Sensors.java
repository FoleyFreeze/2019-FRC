package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;

public class Sensors extends Component {
    public Encoder[] wheelAngles = new Encoder[4];

    public Sensors() {
        wheelAngles[0] = new Encoder(2,3);//practice bot values
        wheelAngles[1] = new Encoder(0,1);
        wheelAngles[2] = new Encoder(4,5);
        wheelAngles[3] = new Encoder(6,7);
        wheelAngles[0].setReverseDirection(true);
        wheelAngles[1].setReverseDirection(true);
        wheelAngles[2].setReverseDirection(true);
        wheelAngles[3].setReverseDirection(true);
    }

    public void run() {
        if(in.resetEncoders){
            wheelAngles[0].reset();
            wheelAngles[1].reset();
            wheelAngles[2].reset();
            wheelAngles[3].reset();
        }
    }
}