package frc.robot.io;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlBoard {

    //2019 Practice board values!!!!!!!!!!!!!!!!!!!!!!!!
    private int IN_high = 13;
    private int IN_middle = 12;
    private int IN_low = 11;
    private int IN_front = 10;

    private int OUT_high = 1; //5
    private int OUT_middle = 12; //4
    private int OUT_low = 11; //3
    private int OUT_front = 10;//2

    private int IN_jogUp = 8;//?????

    private int IN_jogDown = 7;//?????
    
    private int IN_gather = 1;
    private int IN_shoot = 4;
    
    private int IN_farRkt = 16;
    private int IN_cargoShip = 14;
    private int IN_nearRkt = 15;

    private int OUT_farRkt = 8;
    private int OUT_cargoShip = 6;
    private int OUT_nearRkt = 9;//TODO: check that this is right for comp board

    private int IN_lOrR = 5; // isn't left right 
    private int IN_AutoOrSemi = 5; // auto is true 
    private int IN_climb = 6;
    
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

    public void read() {
        
        jogUp = joy.getRawButton(IN_jogUp);
        jogDown = joy.getRawButton(IN_jogDown);
        gather = joy.getRawButton(IN_gather);
        shoot = joy.getRawButton(IN_shoot);
        lOrR = joy.getRawButton(IN_lOrR);
        autoOrSemi = joy.getRawButton(IN_AutoOrSemi);
        climb = joy.getRawButton(IN_climb);
        shift = joy.getRawButton(IN_shift);
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
        if(joy.getRawButton(IN_high)){
            rocketCargoState = RocketCargoshipPosition.HI;
            
        }else if(joy.getRawButton(IN_middle)){
            rocketCargoState = RocketCargoshipPosition.MID;
            
        }else if(joy.getRawButton(IN_low)){
            rocketCargoState = RocketCargoshipPosition.LO;
            
        }else if(joy.getRawButton(IN_front)){
            rocketCargoState = RocketCargoshipPosition.FRONT;
            
        }

        joy.setOutput(OUT_shift, shift);

        switch(rocketCargoState){
            case HI:
            joy.setOutput(OUT_high, true);
            joy.setOutput(OUT_middle, false);
            joy.setOutput(OUT_low, false);
            joy.setOutput(OUT_front, false);
            break;

            case MID:
            joy.setOutput(OUT_high, false);
            joy.setOutput(OUT_middle, true);
            joy.setOutput(OUT_low, false);
            joy.setOutput(OUT_front, false);
            break;

            case LO:
            joy.setOutput(OUT_high, false);
            joy.setOutput(OUT_middle, false);
            joy.setOutput(OUT_low, true);
            joy.setOutput(OUT_front, false);
            break;

            case FRONT:
            joy.setOutput(OUT_high, false);
            joy.setOutput(OUT_middle, false);
            joy.setOutput(OUT_low, false);
            joy.setOutput(OUT_front, true);
            break;   
            
            case DEFAULT:
            joy.setOutput(OUT_high, true);
            joy.setOutput(OUT_middle, true);
            joy.setOutput(OUT_low, true);
            joy.setOutput(OUT_front, true);
            break;
        }
        

        if(joy.getRawButton(IN_farRkt)){
            nearFarCargo = NearFarCargo.FAR;
            
        }else if(joy.getRawButton(IN_nearRkt)){
            nearFarCargo = NearFarCargo.NEAR;
            
        }else if(joy.getRawButton(IN_cargoShip)){
            nearFarCargo = NearFarCargo.CARGO;
            
        }

        switch(nearFarCargo){
            case FAR:
            joy.setOutput(OUT_farRkt, true);
            joy.setOutput(OUT_nearRkt, false);
            joy.setOutput(OUT_cargoShip, false);
            break;

            case NEAR:
            joy.setOutput(OUT_farRkt, false);
            joy.setOutput(OUT_nearRkt, true);
            joy.setOutput(OUT_cargoShip, false);
            break;

            case CARGO:
            joy.setOutput(OUT_farRkt, false);
            joy.setOutput(OUT_nearRkt, false);
            joy.setOutput(OUT_cargoShip, true);
            break;  

            case DEFAULT:
            joy.setOutput(OUT_farRkt, true);
            joy.setOutput(OUT_nearRkt, true);
            joy.setOutput(OUT_cargoShip, true);
            break;
        }

        SmartDashboard.putString("RocketState",rocketCargoState.name());
        SmartDashboard.putString("NearFarCargoState",nearFarCargo.name());
    }


}