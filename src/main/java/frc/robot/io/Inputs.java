package frc.robot.io;

import edu.wpi.first.wpilibj.Joystick;

public class Inputs {
   
    private final Joystick controlBoard;
        private final Joystick gamePad;
        
    public Inputs() {
        
        controlBoard = new Joystick(0);
        gamePad = new Joystick(1);


	//----------------------------------Driver Functions--------------------------------
        public double drive; 
        public double turnRobot;
        public boolean driveStraight = false;
        public boolean fieldOrintation;
    //----------------------------Operator Functions-------------------------------------- 
        public boolean discGather;
        public boolean ballGather;
        public boolean releaseBall; 
        public boolean realeseDisc;
        public boolean climb;
        public boolean reverseclimb;
        public boolean elevatorUp;
        public boolean elevatorDown; 

        public int elevatorTarget = 0;
        
    

    }

    public void run() {
        
    }
}