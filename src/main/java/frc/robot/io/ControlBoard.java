package frc.robot.io;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Component;

public class ControlBoard extends Component{

    //2018 Practice board values!!!!!!!!!!!!!!!!!!!!!!!!
    public int high;
    public int middle;
    public int low;

    public int jogUp;//?????
    public int jogDown;//?????
    
    public int gather;
    public int shoot;
    
    public int farRkt;
    public int cargoShip;
    public int nearRkt;

    public int lOrR;

    public int front;

    public int climb;
    
    public int shift;
    public int pitMode;

    public int ballOrHatch;
    
    public void set(){
        shift = 15;
        ballOrHatch = 12;
        high = 4;
        middle = 3;
        low = 2;
        front = 5;

        gather = 7;
        shoot = 10;

        lOrR = 14;
        
        farRkt = 6;
        cargoShip = 8;
        nearRkt = 9; 

        climb = 1;

        pitMode = 13;
    }
}