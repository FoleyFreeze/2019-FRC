package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.ControlBoard;
import frc.robot.io.ElectroJendz;
import frc.robot.io.ControlBoard.RocketCargoshipPosition;
import frc.robot.util.LED_Driver_Table;

public class LEDs extends Component{
    private Solenoid lowerLED_Left;
    private Solenoid gearFlake_R;
    private Solenoid gearFlake_G;
    private Solenoid gearFlake_B;
    private Solenoid lowerLED_Right;
    private Solenoid camLED;
    private Spark mainLeds;
    private Solenoid cargoLED;

    public LEDs(){
        lowerLED_Left = new Solenoid(ElectroJendz.lowerLeftLED);
        lowerLED_Right = new Solenoid(ElectroJendz.lowerRightLED);
        gearFlake_R = new Solenoid(ElectroJendz.gearFlake_R);
        gearFlake_B = new Solenoid(ElectroJendz.gearFlake_B);
        gearFlake_G = new Solenoid(ElectroJendz.gearFlake_G);
        camLED = new Solenoid(ElectroJendz.camLED);
        mainLeds = new Spark(0);
        cargoLED = new Solenoid(ElectroJendz.cargoLED);
    }

    private double BLINK_TIME_ON = 0.10;
    private double BLINK_TIME_OFF = 0.05;
    private int BLINK_COUNT = 5;
    private double FAST_BLINK_TIME_ON = 0.05;
    private double FAST_BLINK_TIME_OFF = 0.025;
    private boolean FAST = false;

    int blinkCount = BLINK_COUNT;
    double blinkTimer;
    boolean blinkState = true;
    boolean prevStateCargo;
    boolean prevStateHatch;

    public void run(){
        //lowerLED_Left.set(true);
        //lowerLED_Right.set(true);
        camLED.set(in.camLightsOn);
        cargoLED.set(!in.camLightsOn);

        SmartDashboard.putBoolean("LEDs", lowerLED_Left.get());

        //rising edge of hasThing
        if(!prevStateCargo && sense.hasCargo || !prevStateHatch && sense.hasHatch){
            blinkCount = 0;
            blinkTimer = Timer.getFPGATimestamp();
        } 

        /*/camera sees a thing
        if(view.goodCargoImage() || view.goodVisionTargetHigh() || view.goodVisionTargetLow()){
            blinkCount = 0;
        }*/

        if(blinkCount < BLINK_COUNT){
            if(!FAST){
                if(blinkState && Timer.getFPGATimestamp() - blinkTimer > BLINK_TIME_ON) {
                    blinkState = false;
                    blinkTimer = Timer.getFPGATimestamp();
                } else if(!blinkState && Timer.getFPGATimestamp() - blinkTimer > BLINK_TIME_OFF){
                    blinkState = true;
                    blinkTimer = Timer.getFPGATimestamp();
                    blinkCount++;
                }
            }else{
                if(blinkState && Timer.getFPGATimestamp() - blinkTimer > FAST_BLINK_TIME_ON) {
                    blinkState = false;
                    blinkTimer = Timer.getFPGATimestamp();
                } else if(!blinkState && Timer.getFPGATimestamp() - blinkTimer > FAST_BLINK_TIME_OFF){
                    blinkState = true;
                    blinkTimer = Timer.getFPGATimestamp();
                    blinkCount++;
                }
            }
        }

        boolean notGathered = !sense.hasCargo && !sense.hasHatch;
        //gearFlake_R.set(sense.hasCargo);
        //gearFlake_G.set(sense.hasHatch);
        gearFlake_R.set(sense.hasCargo && blinkState);
        gearFlake_G.set(sense.hasHatch && blinkState);
        gearFlake_B.set(notGathered && blinkState);
        //lowerLED_Left.set(blinkState || notGathered);
        //lowerLED_Right.set(blinkState || notGathered);
        lowerLED_Left.set(blinkState);
        lowerLED_Right.set(blinkState);

        prevStateCargo = sense.hasCargo;
        prevStateHatch = sense.hasHatch;

        setMains();
    }

    boolean climbLatch;

    private void setMains(){
        //rainbow when climbing
        //white when shooting
        //red when cargo
        //green when hatch
        //blue otherwise

        if(in.climb && !sense.isDisabled || climbLatch){
            climbLatch = !sense.hasHatchEdge && !sense.hasCargoEdge;
            mainLeds.set(LED_Driver_Table.Fixed_Palette_Pattern_Rainbow_Rainbow);
        } else if(drive.autoShoot){
            mainLeds.set(LED_Driver_Table.Solid_Colors_White);
        } else if(sense.hasCargo){
            /*if(in.leftNotRight){
                if(in.controlBoard.rocketCargoState == RocketCargoshipPosition.HI){
                    FAST = true;
                    if(blinkState)mainLeds.set(LED_Driver_Table.Solid_Colors_Dark_red);
                    else mainLeds.set(LED_Driver_Table.Solid_Colors_Black);
                }else if(in.controlBoard.rocketCargoState == RocketCargoshipPosition.MID){
                    FAST = false;
                    if(blinkState)mainLeds.set(LED_Driver_Table.Solid_Colors_Red);
                    else mainLeds.set(LED_Driver_Table.Solid_Colors_Black);
                }else if(in.controlBoard.rocketCargoState == RocketCargoshipPosition.LO || in.controlBoard.rocketCargoState == RocketCargoshipPosition.FRONT){
                    FAST = false;
                    mainLeds.set(LED_Driver_Table.Solid_Colors_Hot_Pink);
                }else{
                    mainLeds.set(LED_Driver_Table.Fixed_Palette_Pattern_Rainbow_Lave);
                }
            }else{
                if(in.controlBoard.rocketCargoState == RocketCargoshipPosition.HI){
                    FAST = true;
                    if(blinkState)mainLeds.set(LED_Driver_Table.Solid_Colors_Dark_red);
                    else mainLeds.set(LED_Driver_Table.Solid_Colors_Black);
                }else if(in.controlBoard.rocketCargoState == RocketCargoshipPosition.MID){
                    FAST = false;
                    if(blinkState)mainLeds.set(LED_Driver_Table.Solid_Colors_Red);
                    else mainLeds.set(LED_Driver_Table.Solid_Colors_Black);
                }else if(in.controlBoard.rocketCargoState == RocketCargoshipPosition.LO || in.controlBoard.rocketCargoState == RocketCargoshipPosition.FRONT){
                    FAST = false;
                    mainLeds.set(LED_Driver_Table.Solid_Colors_Hot_Pink);
                }else{
                    mainLeds.set(LED_Driver_Table.Fixed_Palette_Pattern_Rainbow_Lave);
                }
            }*/
            mainLeds.set(LED_Driver_Table.Fixed_Palette_Pattern_Rainbow_Lave);
        } else if(sense.hasHatch){
           /* if(in.controlBoard.rocketCargoState == RocketCargoshipPosition.HI){
                FAST = true;
                if(blinkState)mainLeds.set(LED_Driver_Table.Solid_Colors_Dark_Green);
                else mainLeds.set(LED_Driver_Table.Solid_Colors_Black);
            }else if(in.controlBoard.rocketCargoState == RocketCargoshipPosition.MID){
                FAST = false;
                if(blinkState)mainLeds.set(LED_Driver_Table.Solid_Colors_Lawn_Green);
                else mainLeds.set(LED_Driver_Table.Solid_Colors_Black);
            }else if(in.controlBoard.rocketCargoState == RocketCargoshipPosition.LO || in.controlBoard.rocketCargoState == RocketCargoshipPosition.FRONT){
                FAST = false;
                mainLeds.set(LED_Driver_Table.Solid_Colors_Lime);
            }else{
                mainLeds.set(LED_Driver_Table.Fixed_Palette_Pattern_Rainbow_Forest);
            }
        } else {*/
            mainLeds.set(LED_Driver_Table.Fixed_Palette_Pattern_Rainbow_Ocean);
        } 
    }
}