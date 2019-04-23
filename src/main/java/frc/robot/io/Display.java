package frc.robot.io;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Component;

public class Display extends Component{

	NetworkTableEntry hasHatch;
	NetworkTableEntry hasCargo;
	NetworkTableEntry rseX;
	NetworkTableEntry rseY;
	NetworkTableEntry eleEnc;
	NetworkTableEntry startSelector;
	NetworkTableEntry currPoly;
	NetworkTableEntry lastDist;
	NetworkTableEntry startLocName;
	NetworkTableEntry hab2Status;
	NetworkTableEntry robotAngle;

    public Display(){

		ShuffleboardTab compTab = Shuffleboard.getTab("CompTab");

        hasHatch = compTab.add("hasHatch",false).withPosition(0,0).getEntry();
        hasCargo = compTab.add("hasCargo",false).withPosition(0,1).getEntry();
        rseX = compTab.add("rseX",0.0).withPosition(2,0).getEntry();
        rseY = compTab.add("rseY",0.0).withPosition(2,1).getEntry();
		eleEnc = compTab.add("EleEnc",0.0).withPosition(3,0).getEntry();
		currPoly = compTab.add("CurrPoly",0).withPosition(1,1).getEntry();
		lastDist = compTab.add("LastRSEdist",0).withPosition(2,2).getEntry();
		startLocName = compTab.add("HabStartLoc","None").withPosition(1,0).getEntry();
		hab2Status = compTab.add("Hab2Start",false).withPosition(0,2).getEntry();
		robotAngle = compTab.add("RobotAngle",0).withPosition(0,3).getEntry();
    }
	
	public void run(){
		hasHatch.setBoolean(sense.hasHatch);
		hasCargo.setBoolean(sense.hasCargo);
		rseX.setDouble(rse.x);
		rseY.setDouble(rse.y);
		eleEnc.setDouble(sense.elevatorEncoder);
		currPoly.setDouble(autoDriving.currPolyId);
		lastDist.setDouble(autoDriving.lastDist);
		startLocName.setString(rse.habStartLoc);
		hab2Status.setBoolean(autoDriving.startingHab2);
		robotAngle.setDouble(sense.robotAngle.getDeg());
	}

}