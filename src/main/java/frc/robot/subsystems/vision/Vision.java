package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.ControlBoard.NearFarCargo;
import frc.robot.subsystems.Component;
import frc.robot.util.Angle;
import frc.robot.util.LimitedStack;
import frc.robot.subsystems.autodrive.Point;

//public class Vision extends Component {

public class Vision extends Component implements PiEvent {

    DigitalInput testInput;

    LimitedStack<VisionData> targetHighStack;
    LimitedStack<VisionData> targetLowStack;
    LimitedStack<VisionData> cargoStack;

    NetworkTable piCommands;
    NetworkTableEntry piFindCargo;
    NetworkTableEntry piFindTargetHigh;
    NetworkTableEntry piFindTargetLow;
    NetworkTableEntry timestamp;

    double lastFrameTime;
    int lastSeqNum;

    double lastIntTime;
    double deltaIntTime;

    public Vision() {   //Vision Constructor
        /*
        lastIntTime = Timer.getFPGATimestamp();
        testInput = new DigitalInput(1);
        testInput.requestInterrupts(new InterruptHandlerFunction<Void>() {
            public void interruptFired(int num, Void v){
                double t = Timer.getFPGATimestamp();
                deltaIntTime = t - lastIntTime;
                lastIntTime = t;
            }
        });*/

        piCommands = NetworkTableInstance.getDefault().getTable("PiControl");
        NetworkTableInstance.getDefault().setUpdateRate(.020);
        piFindCargo = piCommands.getEntry("FindCargo");
        piFindTargetHigh = piCommands.getEntry("FindTargetHigh");
        piFindTargetLow = piCommands.getEntry("FindTargetLow");
        timestamp = piCommands.getEntry("Timestamp");

        targetHighStack = new LimitedStack<>(5);
        targetLowStack = new LimitedStack<>(5);
        cargoStack = new LimitedStack<>(5);

        // Start listening for messages from the Pi
        PiListener pl = new PiListener(this, 65432, "10.9.10.2");
        Thread t = new Thread(null, pl, "PiListener");
        t.start();

    } //end of Vision Constructor

    public void eventGet(String s) {
        if(!k.CAM_Disabled){ // if CAM is NOT disabled...

            // ***********  H I G H  ***********

            if (cmdHighTarget) {
                VisionData vd = new VisionData();
                SmartDashboard.putString("VisionTargetHigh", s);
                String[] parts = s.split(",");

                vd.distance = Double.parseDouble(parts[2]);
                vd.angleTo = Double.parseDouble(parts[3]);
                vd.angleOf = Double.parseDouble(parts[4]);

                vd.timeStamp = Timer.getFPGATimestamp();
                vd.robotAngle = sense.robotAngle.getDeg();

                vd.dt = vd.timeStamp - lastFrameTime;
                SmartDashboard.putNumber("Image dt",vd.dt);
                lastFrameTime = vd.timeStamp;

                VisionData oldData = targetHighStack.peek();
                if(vd.dt < 0.2 && oldData != null){
                    vd.dDist = oldData.distance - vd.distance;
                    vd.dAngle = oldData.angleTo - vd.angleTo;
                    if(vd.dAngle > 180) vd.dAngle += 360;
                    if(vd.dAngle < -180) vd.dAngle -= 360;
                } else {
                    vd.dDist = 0;
                    vd.dAngle = 0;
                }

                //saving the RSE values from when the picture was recieved
                //vd.rseX = rse.x;
                //vd.rseY = rse.y;

                double captureTime = Double.parseDouble(parts[0]);
                double processTime = Double.parseDouble(parts[1]);
                double latency = (vd.timeStamp - captureTime - processTime) / 2;
                SmartDashboard.putNumber("VisionLatency",latency);
                Point p = rse.getPositionAtTime(vd.timeStamp - latency - processTime);
                vd.rseX = p.x;
                vd.rseY = p.y;
                                
                targetHighStack.push(vd);
            } //End of HIGH
            
            // ***********  L O W  ***********

            boolean hatch_in_the_way; //flat do test if cargo is going to block the camera
            //don't trust the camera when hatch panel is in the way
            hatch_in_the_way = (!in.cargoNotHatch && sense.elevatorEncoder > 16 && sense.elevatorEncoder < 35) ||
                               (!k.SCR_ScorpioSelected && in.cargoNotHatch && sense.elevatorEncoder > 20 && sense.elevatorEncoder < 33);

            if (cmdLowTarget && !hatch_in_the_way) { //If LOW and No elevator blocking us
                VisionData vd = new VisionData();
                SmartDashboard.putString("VisionTargetHigh", s);
                String[] parts = s.split(",");

                vd.distance = Double.parseDouble(parts[2]);
                vd.angleTo = Double.parseDouble(parts[3]);
                vd.angleOf = Double.parseDouble(parts[4]);

                vd.timeStamp = Timer.getFPGATimestamp();
                vd.robotAngle = sense.robotAngle.getDeg();

                double distX = (vd.distance + 16) * Math.tan(Angle.toRad(vd.angleTo));
                double distY = vd.distance;
                
                SmartDashboard.putNumber("vdX" , distX);
                SmartDashboard.putNumber("vdY", distY);

                vd.dt = vd.timeStamp - lastFrameTime;
                SmartDashboard.putNumber("Image dt",vd.dt);
                lastFrameTime = vd.timeStamp;

                VisionData oldData = targetLowStack.peek();
                if(vd.dt < 0.2 && oldData != null){
                    vd.dDist = oldData.distance - vd.distance;
                    vd.dAngle = oldData.angleTo - vd.angleTo;
                    if(vd.dAngle > 180) vd.dAngle += 360;
                    if(vd.dAngle < -180) vd.dAngle -= 360;
                } else {
                    vd.dDist = 0;
                    vd.dAngle = 0;
                }

                //saving the RSE values from when the picture was recieved
                //vd.rseX = rse.x;
                //vd.rseY = rse.y;

                double captureTime = Double.parseDouble(parts[0]);
                double processTime = Double.parseDouble(parts[1]);
                double latency = (vd.timeStamp - captureTime - processTime) / 2;
                SmartDashboard.putNumber("VisionLatency",latency);
                Point p = rse.getPositionAtTime(vd.timeStamp - latency - processTime);
                vd.rseX = p.x;
                vd.rseY = p.y;

                targetLowStack.push(vd);

            } // end of LOW


            // ***********  C A R G O  ***********
            
            if (cmdCargoTarget) {
                VisionData vd = new VisionData();
                SmartDashboard.putString("Cargo-Data", s);
                String[] parts = s.split(",");

                vd.distance = Double.parseDouble(parts[2]);
                vd.angleTo = Double.parseDouble(parts[3]);
                vd.angleOf = 0; //default 0 because angleOf is now a slope internally
                
                vd.timeStamp = Timer.getFPGATimestamp();
                vd.robotAngle = sense.robotAngle.getDeg();

                vd.dt = vd.timeStamp - lastFrameTime;
                SmartDashboard.putNumber("Image dt",vd.dt);
                lastFrameTime = vd.timeStamp;

                VisionData oldData = cargoStack.peek();
                if(vd.dt < 0.2 && oldData != null){
                    vd.dDist = oldData.distance - vd.distance;
                    vd.dAngle = oldData.angleTo - vd.angleTo;
                    if(vd.dAngle > 180) vd.dAngle += 360;
                    if(vd.dAngle < -180) vd.dAngle -= 360;
                } else {
                    vd.dDist = 0;
                    vd.dAngle = 0;
                }

                double captureTime = Double.parseDouble(parts[0]);
                double processTime = Double.parseDouble(parts[1]);
                double latency = (vd.timeStamp - captureTime - processTime) / 2;
                SmartDashboard.putNumber("VisionLatency",latency);
                Point p = rse.getPositionAtTime(vd.timeStamp - latency - processTime);
                vd.rseX = p.x;
                vd.rseY = p.y;
                
                cargoStack.push(vd);

            }// End of CARGO 


        }//End of IF CAM not disabled

    }//End of eventGet()


    public boolean lastTargetsGood(int count){
        //determine if we are looking for high or low targets
        LimitedStack<VisionData> stack;
        if(in.scoringCargo && in.controlBoard.nearFarCargo != NearFarCargo.CARGO){
            //looking for high targets
            stack = targetHighStack;
        } else {
            //looking for low targets
            stack = targetLowStack;
        }

        //check that we have seen the target 3 times in rapid succession
        double timeLimit = 0.125 * count;
        return stack.size() >= count && Timer.getFPGATimestamp() - stack.get(count-1).timeStamp < timeLimit;
    }

    public boolean goodCargoImage(){
        VisionData vd = cargoStack.peek();
        return vd != null && Timer.getFPGATimestamp() - vd.timeStamp < k.CAM_ExpireTime;
    }

    public boolean goodVisionTargetHigh(){
        //TODO: eventually check the image vs the expected location of the target
        VisionData vd = targetHighStack.peek();
        return vd != null && Timer.getFPGATimestamp() - vd.timeStamp < k.CAM_ExpireTime; 
    }

    public boolean goodVisionTargetLow(){
        //TODO: eventually check the image vs the expected location of the target
        VisionData vd = targetLowStack.peek();
        return vd != null && Timer.getFPGATimestamp() - vd.timeStamp < k.CAM_ExpireTime; 
    }

    public VisionData getLastCargo(){
        return cargoStack.peek();
    }

    public VisionData getLastVisionTargetHigh(){
        return targetHighStack.peek();
    }

    public VisionData getLastVisionTargetLow(){
        return targetLowStack.peek();
    }

    public boolean cmdHighTarget;
    public boolean cmdLowTarget;
    public boolean cmdCargoTarget;
    public void run(){
        boolean PiHighSearch = in.scoringCargo && in.controlBoard.nearFarCargo != NearFarCargo.CARGO;
        cmdCargoTarget = in.searchingCargo || k.CAM_DebugCargo;
        piFindCargo.setBoolean(cmdCargoTarget);
        cmdHighTarget = PiHighSearch || k.CAM_DebugTargetHigh;
        piFindTargetHigh.setBoolean(cmdHighTarget);//(in.visionTargetHigh || k.CAM_DebugTargetHigh);//
        cmdLowTarget = (!PiHighSearch && !in.searchingCargo && !sense.isDisabled) || k.CAM_DebugTargetLow;
        piFindTargetLow.setBoolean(cmdLowTarget);//(in.visionTargetLow || k.CAM_DebugTargetLow);//(in.searchingHatch || k.CAM_DebugTargetLow);
        
        timestamp.setNumber(Timer.getFPGATimestamp());
        
        //piFindTargetHigh.setBoolean(in.scoringCargo || k.CAM_DebugTargetHigh);
        //piFindTargetLow.setBoolean(in.searchingHatch || k.CAM_DebugTargetLow);

        //SmartDashboard.putNumber("InterruptDT", deltaIntTime);
    }
}