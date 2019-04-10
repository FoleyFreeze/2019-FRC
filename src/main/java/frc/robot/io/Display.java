package frc.robot.io;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

    public Display(){
        hasHatch = Shuffleboard.getTab("CompTab").add("hasHatch",false).getEntry();
        hasCargo = Shuffleboard.getTab("CompTab").add("hasCargo",false).getEntry();
        rseX = Shuffleboard.getTab("CompTab").add("rseX",0.0).getEntry();
        rseY = Shuffleboard.getTab("CompTab").add("rseY",0.0).getEntry();
		eleEnc = Shuffleboard.getTab("CompTab").add("EleEnc",0.0).getEntry();
		currPoly = Shuffleboard.getTab("CompTab").add("CurrPoly",0).getEntry();
		lastDist = Shuffleboard.getTab("CompTab").add("LastRSEdist",0).getEntry();
		//Shuffleboard.getTab("CompTab").add("StartLocation",rse.startSelector);
		startLocName = Shuffleboard.getTab("CompTab").add("HabStartLoc","None").getEntry();
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
	}

}