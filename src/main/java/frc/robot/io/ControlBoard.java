package frc.robot.io;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Component;

public class ControlBoard extends Component{

    //2019 Practice board values!!!!!!!!!!!!!!!!!!!!!!!!
    public int high = 12;
    public int middle = 11;
    public int low = 10;
    public int front = 9;

    public int highOut = 5;
    public int middleOut = 4;
    public int lowOut = 3;
    public int frontOut = 2;

    public int jogUp = 8;//?????
    public int jogDown = 7;//?????
    
    public int gather = 1;
    public int shoot = 4;
    
    public int farRkt = 15;
    public int cargoShip = 13;
    public int nearRkt = 14;

    public int farRktOut = 8;
    public int cargoShipOut = 6;
    public int nearRktOut = 9;//TODO: check that this is right for comp board

    public int lOrR = 5;

    public int climb = 6;
    
    public int shift = 3;
    public int shiftOut = 1;
    public int pitMode = 16;

    public int cargoOrHatch = 2;
}