package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.ElectroJendz;

public class LEDs extends Component{
    private Solenoid lowerLED_Left;
    private Solenoid gearFlake_R;
    private Solenoid gearFlake_G;
    private Solenoid gearFlake_B;
    private Solenoid lowerLED_Right;
    private Solenoid camLED;

    public LEDs(){
        lowerLED_Left = new Solenoid(ElectroJendz.lowerLeftLED);
        lowerLED_Right = new Solenoid(ElectroJendz.lowerRightLED);
        gearFlake_R = new Solenoid(ElectroJendz.gearFlake_R);
        gearFlake_B = new Solenoid(ElectroJendz.gearFlake_B);
        gearFlake_G = new Solenoid(ElectroJendz.gearFlake_G);
        camLED = new Solenoid(ElectroJendz.camLED);
    }

    private double BLINK_TIME_ON = 0.20;
    private double BLINK_TIME_OFF = 0.05;
    private int BLINK_COUNT = 5;

    int blinkCount = BLINK_COUNT;
    double blinkTimer;
    boolean blinkState;
    boolean prevStateCargo;
    boolean prevStateHatch;

    public void run(){
        lowerLED_Left.set(true);
        lowerLED_Right.set(true);
        camLED.set(in.camLightsOn);

        SmartDashboard.putBoolean("LEDs", lowerLED_Left.get());

        //rising edge of hasThing
        if(!prevStateCargo && sense.hasCargo || !prevStateHatch && sense.hasHatch){
            blinkCount = 0;
            blinkTimer = Timer.getFPGATimestamp();
        } 

        if(blinkCount < BLINK_COUNT){
            if(blinkState && Timer.getFPGATimestamp() - blinkTimer > BLINK_TIME_ON) {
                blinkState = false;
                blinkTimer = Timer.getFPGATimestamp();
            } else if(!blinkState && Timer.getFPGATimestamp() - blinkTimer > BLINK_TIME_OFF){
                blinkState = true;
                blinkTimer = Timer.getFPGATimestamp();
                blinkCount++;
            }
        }

        //gearFlake_R.set(sense.hasCargo);
        //gearFlake_G.set(sense.hasHatch);
        gearFlake_R.set(sense.hasCargo && blinkState);
        gearFlake_G.set(sense.hasHatch && blinkState);
        gearFlake_B.set(!sense.hasCargo && !sense.hasHatch);

        prevStateCargo = sense.hasCargo;
        prevStateHatch = sense.hasHatch;

    }
}