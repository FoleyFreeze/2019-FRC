
package frc.robot.io;

import edu.wpi.first.wpilibj.Joystick;

public class Controller {
    
    private int IN_resetGyro;
    private int IN_compassDrive;
    private int IN_fieldOriented;
    private int IN_flipOrientation;
    private int IN_pitMode;
    private int IN_hatchGather;
    private boolean IN_flySky = false;
    private int IN_resetElePart1;
    private int IN_resetElePart2;
    public double IN_xDeadband;
    public double IN_yDeadband;
    public double IN_rotDeadband;
    public double IN_xyDeadband;
    private int IN_xDriveAxis;
    private int IN_yDriveAxis;
    private int IN_rotDriveAxis;
    private int IN_dodgingL = 2;
    private int IN_dodgingR = 3;

    public boolean resetGyro;
    public boolean resetElePart1;
    public boolean resetElePart2;
    public boolean compassDrive;
    public boolean fieldOriented;
    public boolean flipOrientation;
    public boolean pitMode;
    public boolean hatchGather;
    public double xDriveAxis;
    public double yDriveAxis;
    public double rotDriveAxis;
    public double leftTrigger;
    public double rightTrigger;

    private Joystick joy;

    public Controller(int port){
        joy = new Joystick(port);
    }

    public void read(){
        //Joystick joy = new Joystick(ElectroJendz.GAMEPAD);
        IN_flySky = joy.getButtonCount() > 10;
        //option 2 for detecting what controller
        //joy.getName().toLowerCase().contains("xbox");
        //System.out.println(joy.getName());

        if(IN_flySky){
            //Buttons if Fly Sky controller
            IN_resetGyro = 10;
            IN_compassDrive = 1;
            IN_fieldOriented = 5;
            IN_flipOrientation = 0;
            IN_pitMode = 0;
            IN_hatchGather = 0;
            IN_xDeadband = 0.05;
            IN_yDeadband = 0.05;
            IN_rotDeadband = 0.05;
            IN_xyDeadband = 0.07;
            IN_xDriveAxis = 0;
            IN_yDriveAxis = 1;
            IN_rotDriveAxis = 4;
            IN_dodgingL = 2;
            IN_dodgingR = 3;
            IN_resetElePart1 = 11;
            IN_resetElePart2 = 15;
        }else{
            IN_resetGyro = 1;
            IN_compassDrive = 5;
            IN_fieldOriented = 6;
            IN_flipOrientation = 0;
            IN_pitMode = 0;
            IN_hatchGather = 0;
            IN_xDeadband = 0.15;
            IN_yDeadband = 0.15;
            IN_rotDeadband = 0.15;
            IN_xyDeadband = 0.2;
            IN_xDriveAxis = 0;
            IN_yDriveAxis = 1;
            IN_rotDriveAxis = 4;
            IN_dodgingL = 2;
            IN_dodgingR = 3;
            IN_resetElePart1 = 0;
            IN_resetElePart2 = 0;
        }

        if(IN_resetGyro != 0) resetGyro = joy.getRawButton(IN_resetGyro);
        else resetGyro = false;

        if(IN_compassDrive != 0) compassDrive = joy.getRawButton(IN_compassDrive);
        else compassDrive = false;

        if(IN_fieldOriented != 0) fieldOriented = joy.getRawButton(IN_fieldOriented);
        else fieldOriented = false;

        if(IN_flipOrientation != 0) flipOrientation = joy.getRawButton(IN_flipOrientation);
        else flipOrientation = false;

        if(IN_pitMode != 0) pitMode = joy.getRawButton(IN_pitMode);
        else pitMode = false;

        if(IN_hatchGather != 0) hatchGather = joy.getRawButton(IN_hatchGather);
        else hatchGather = false;

        if(IN_xDriveAxis != -1) xDriveAxis = joy.getRawAxis(IN_xDriveAxis);
        else xDriveAxis = 0;

        if(IN_yDriveAxis != -1) yDriveAxis = joy.getRawAxis(IN_yDriveAxis);
        else yDriveAxis = 0;

        if(IN_rotDriveAxis != -1) rotDriveAxis = joy.getRawAxis(IN_rotDriveAxis);
        else rotDriveAxis = 0; 

        if(IN_dodgingL != -1) leftTrigger = joy.getRawAxis(IN_dodgingL); 
        else leftTrigger = 0;

        if(IN_dodgingR != -1) rightTrigger = joy.getRawAxis(IN_dodgingR);
        else rightTrigger = 0;

        if(IN_resetElePart1 != 0) resetElePart1 = joy.getRawButton(IN_resetElePart1);
        else resetElePart1 = false;

        if(IN_resetElePart2 != 0) resetElePart2 = joy.getRawButton(IN_resetElePart2);
        else resetElePart2 = false;
    }

} 
