
package frc.robot.io;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonMap {
    
    public int resetGyro;
    public int compassDrive;
    public int fieldOriented;
    public int flipOrientation;
    public int pitMode;
    public int hatchGather;
    public boolean flySky = false;
    public double xDeadband;
    public double yDeadband;
    public double rotDeadband;
    public double xyDeadband;
    public int IN_xDriveAxis;
    public int IN_yDriveAxis;
    public int IN_rotDriveAxis;
    public int IN_dodgingL = 2;
    public int IN_dodgingR = 3;

    public void set(){
        Joystick joy = new Joystick(ElectroJendz.GAMEPAD);
        flySky = joy.getButtonCount() > 10;
        //option 2 for detecting what controller
        //joy.getName().toLowerCase().contains("xbox");
        //System.out.println(joy.getName());

        if(flySky){
            //Buttons if Fly Sky controller
            resetGyro = 10;
            compassDrive = 1;
            fieldOriented = 5;
            flipOrientation = -0;
            pitMode = -0;
            hatchGather = -0;
            xDeadband = 0.05;
            yDeadband = 0.05;
            rotDeadband = 0.05;
            xyDeadband = 0.07;
            IN_xDriveAxis = 0;
            IN_yDriveAxis = 1;
            IN_rotDriveAxis = 4;
            IN_dodgingL = 2;
            IN_dodgingR = 3;
        }else{
            resetGyro = 1;
            compassDrive = 5;
            fieldOriented = 6;
            flipOrientation = -0;
            pitMode = -0;
            hatchGather = -0;
            xDeadband = 0.05;
            yDeadband = 0.05;
            rotDeadband = 0.1;
            xyDeadband = 0.1;
            IN_xDriveAxis = 0;
            IN_yDriveAxis = 1;
            IN_rotDriveAxis = 4;
            IN_dodgingL = 2;
            IN_dodgingR = 3;
        }
    }

} 
