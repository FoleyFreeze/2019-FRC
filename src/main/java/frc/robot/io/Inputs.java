package frc.robot.io;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Elevator;

public class Inputs {
   
    private final Joystick controlBoard;
    private final Joystick gamePad;
        
    //----------------------------------Driver Functions--------------------------------
    public double xAxisDrive; 
    public double yAxisDrive;
    public double rotAxisDrive;
    public boolean fieldOrientation;
    public boolean resetEncoders;
    public boolean fieldOriented;
    public boolean compassDrive;
    //----------------------------Operator Functions-------------------------------------- 
    public boolean ballGather;
    public boolean releaseBall; 
    public boolean discGather;
    public boolean releaseDisc; 

    public boolean climb;
    public boolean reverseclimb;
<<<<<<< HEAD

    public boolean manualElevatorUp;
    public boolean manualElevatorDown; 
    public boolean autoElevator;
    public Elevator.ElevatorPosition elevatorTarget;
    //elevatorTarget means you press a button and it moves to a specific place
    
=======

    public boolean elevatorUp;
    public boolean elevatorDown; 
    public int elevatorTarget = 0;




>>>>>>> R.B. worked on gatherer stuff
    public Inputs() {
        controlBoard = new Joystick(0);
        gamePad = new Joystick(1);
    }

    

    public void run() {
        if(gamePad.getRawButton(1)){
            resetEncoders = true;
        }else{
            resetEncoders = false;
        }
     
        //read joystick
        xAxisDrive = gamePad.getRawAxis(0);
        yAxisDrive = -gamePad.getRawAxis(1);
        rotAxisDrive = -gamePad.getRawAxis(4);

        //deadband if tiny
        if(Math.abs(xAxisDrive) < 0.05) xAxisDrive = 0; 
        if(Math.abs(yAxisDrive) < 0.05) yAxisDrive = 0; 
        if(Math.abs(xAxisDrive) < 0.2 && Math.abs(yAxisDrive) < 0.2) {
            xAxisDrive = 0;
            yAxisDrive = 0;
        }
        if(Math.abs(rotAxisDrive) < 0.2) rotAxisDrive = 0;

        if(gamePad.getTriggerPressed()){
            compassDrive = true;
        }

        if(compassDrive){
            compassDrive();
        }
    }

<<<<<<< HEAD
    public void compassDrive(){
        double theta = Math.atan2(yAxisDrive, xAxisDrive);
        double r = Math.sqrt(xAxisDrive * xAxisDrive + yAxisDrive * yAxisDrive);
        theta = theta/45;
        Math.round(theta);
        theta = theta * 45;
        yAxisDrive = r * Math.sin(theta);
        xAxisDrive = r * Math.cos(theta);
    }
=======
>>>>>>> R.B. worked on gatherer stuff

}