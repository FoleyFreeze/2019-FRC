package frc.robot.io;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlBoard {

    //2019 Practice board values!!!!!!!!!!!!!!!!!!!!!!!!
    private int IN_high = 13;
    private int IN_middle = 12;
    private int IN_low = 11;
    private int IN_front = 10;

    private int OUT_high = 2; //5
    private int OUT_middle = 12; //4
    private int OUT_low = 11; //3
    private int OUT_front = 10;//2

    private int IN_jogUp = 8;//?????

    private int IN_jogDown = 7;//?????
    
    private int IN_gather = 1;
    private int OUT_gather = 1;
    private int IN_shoot = 4;
    private int OUT_shoot = 4;
    
    private int IN_farRkt = 16;
    private int IN_cargoShip = 14;
    private int IN_nearRkt = 15;

    private int OUT_farRkt = 9;
    private int OUT_cargoShip = 7;
    private int OUT_nearRkt = 8;

    private int IN_lOrR = 17; // isn't left right 
    private int IN_AutoOrSemi = 5; // auto is true 
    private int IN_climb = 6;
    private int OUT_climb = 6;
    
    private int IN_shift = 3;
    private int OUT_shift = 3;
    private int IN_pitMode = 9; //16 

    private int IN_cargoOrHatch = 2;
    
    public boolean jogUp;
    public boolean jogDown;
    public boolean gather;
    public boolean shoot;
    public boolean lOrR;
    public boolean autoOrSemi;
    public boolean climb; 
    public boolean shift; 
    public boolean pitMode; 
    public boolean cargoOrHatch;

    private Joystick joy;

    public ControlBoard(int port){
        joy = new Joystick(port);
    }

    public void read(boolean hasThing) {

        jogUp = joy.getRawButton(IN_jogUp);
        jogDown = joy.getRawButton(IN_jogDown);
        gather = joy.getRawButton(IN_gather);
        joy.setOutput(OUT_gather, hasThing);
        shoot = joy.getRawButton(IN_shoot);
        joy.setOutput(OUT_shoot, hasThing);
        lOrR = joy.getRawButton(IN_lOrR);
        autoOrSemi = joy.getRawButton(IN_AutoOrSemi);
        climb = joy.getRawButton(IN_climb);
        joy.setOutput(OUT_climb, climb);
        shift = joy.getRawButton(IN_shift);
        joy.setOutput(OUT_shift, hasThing);
        pitMode = joy.getRawButton(IN_pitMode);
        cargoOrHatch = joy.getRawButton(IN_cargoOrHatch);
        parseControlBoard();
    }

    public enum RocketCargoshipPosition {
        HI, MID, LO, FRONT, DEFAULT
    }
    public RocketCargoshipPosition rocketCargoState;
    public enum NearFarCargo {
        NEAR, FAR, CARGO, DEFAULT
    }
    public NearFarCargo nearFarCargo;

    private void parseControlBoard(){

        int count = 0;
        if(joy.getRawButton(IN_low)){
            rocketCargoState = RocketCargoshipPosition.LO;
            count++;
        }
        if(joy.getRawButton(IN_high)){
            rocketCargoState = RocketCargoshipPosition.HI;
            count++;
        }
        if(joy.getRawButton(IN_middle)){
            rocketCargoState = RocketCargoshipPosition.MID;
            count++;
        }
        if(joy.getRawButton(IN_front)){
            rocketCargoState = RocketCargoshipPosition.FRONT;
            count++;
        }
        if(count >= 3){
            rocketCargoState = RocketCargoshipPosition.DEFAULT;
        }

        joy.setOutput(OUT_shift, shift);

        joy.setOutput(OUT_high, rocketCargoState == RocketCargoshipPosition.HI 
                        || rocketCargoState == RocketCargoshipPosition.DEFAULT);
        joy.setOutput(OUT_middle, rocketCargoState == RocketCargoshipPosition.MID 
                        || rocketCargoState == RocketCargoshipPosition.DEFAULT);
        joy.setOutput(OUT_low, rocketCargoState == RocketCargoshipPosition.LO 
                        || rocketCargoState == RocketCargoshipPosition.DEFAULT);
        joy.setOutput(OUT_front, rocketCargoState == RocketCargoshipPosition.FRONT 
                        || rocketCargoState == RocketCargoshipPosition.DEFAULT);

        count = 0;
        if(joy.getRawButton(IN_nearRkt)){
            nearFarCargo = NearFarCargo.NEAR;
            count++;
        }
        if(joy.getRawButton(IN_farRkt)){
            nearFarCargo = NearFarCargo.FAR;
            count++;
        }
        if(joy.getRawButton(IN_cargoShip)){
            nearFarCargo = NearFarCargo.CARGO;
            count++;
        }
        if(count >= 3){
            nearFarCargo = NearFarCargo.DEFAULT;
        }

        joy.setOutput(OUT_farRkt, nearFarCargo == NearFarCargo.FAR 
                        || nearFarCargo == NearFarCargo.DEFAULT);
        joy.setOutput(OUT_nearRkt, nearFarCargo == NearFarCargo.NEAR 
                        || nearFarCargo == NearFarCargo.DEFAULT);
        joy.setOutput(OUT_cargoShip, nearFarCargo == NearFarCargo.CARGO 
                        || nearFarCargo == NearFarCargo.DEFAULT);

        SmartDashboard.putString("RocketState",rocketCargoState.name());
        SmartDashboard.putString("NearFarCargoState",nearFarCargo.name());
    }


}