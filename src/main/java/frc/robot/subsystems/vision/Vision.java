package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Component;
import frc.robot.util.LimitedStack;

public class Vision extends Component {

    LimitedStack<VisionData> targetHighStack;
    LimitedStack<VisionData> targetLowStack;
    LimitedStack<VisionData> cargoStack;

    NetworkTable piCommands;
    NetworkTableEntry piFindCargo;
    NetworkTableEntry piFindTargetHigh;
    NetworkTableEntry piFindTargetLow;

    public Vision() {
        piCommands = NetworkTableInstance.getDefault().getTable("PiControl");
        piFindCargo = piCommands.getEntry("FindCargo");
        piFindTargetHigh = piCommands.getEntry("FindTargetHigh");
        piFindTargetLow = piCommands.getEntry("FindTargetLow");

        targetHighStack = new LimitedStack<>(5);
        targetLowStack = new LimitedStack<>(5);
        cargoStack = new LimitedStack<>(5);
        if(!k.CAM_Disabled){
            NetworkTable nt = NetworkTableInstance.getDefault().getTable("Vision");
            nt.addEntryListener("vis_target_high", (table,key,entry,value,flags) -> {
                try{
                    VisionData vd = new VisionData();
                    String data = value.getString(); //seqNum, dist, angleTo, angleOf, etc?
                    SmartDashboard.putString("VisionTargetHigh", data);
                    String[] parts = data.split(",");

                    vd.distance = Double.parseDouble(parts[1]);
                    vd.angleTo = Double.parseDouble(parts[2]);
                    vd.angleOf = Double.parseDouble(parts[3]);

                    vd.timeStamp = Timer.getFPGATimestamp();
                    vd.robotAngle = sense.robotAngle.getDeg();
                    
                    //transform the camera distance vector into a field relative position
                    double camRad = vd.angleTo * Math.PI / 180;
                    double robotX = -vd.distance * Math.sin(camRad) + k.CAM_Location_X;
                    double robotY = vd.distance * Math.cos(camRad) + k.CAM_Location_Y;
                    vd.targetX = robotX * Math.cos(sense.robotAngle.getRad()) - robotY * Math.sin(sense.robotAngle.getRad()) + rse.x;
                    vd.targetY = robotX * Math.sin(sense.robotAngle.getRad()) + robotY * Math.cos(sense.robotAngle.getRad()) + rse.y;

                    targetHighStack.push(vd);
                } catch(Exception e){
                    e.printStackTrace();
                }
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            nt.addEntryListener("vis_target_low", (table,key,entry,value,flags) -> {
                try{
                    VisionData vd = new VisionData();
                    String data = value.getString(); //seqNum, dist, angleTo, angleOf, etc?
                    SmartDashboard.putString("VisionTargetLow", data);
                    String[] parts = data.split(",");

                    vd.distance = Double.parseDouble(parts[1]);
                    vd.angleTo = Double.parseDouble(parts[2]);
                    vd.angleOf = Double.parseDouble(parts[3]);

                    vd.timeStamp = Timer.getFPGATimestamp();
                    vd.robotAngle = sense.robotAngle.getDeg();
                    
                    /*/transform the camera distance vector into a field relative position
                    double camRad = vd.angleTo * Math.PI / 180;
                    double robotX = -vd.distance * Math.sin(camRad) + k.CAM_Location_X;
                    double robotY = vd.distance * Math.cos(camRad) + k.CAM_Location_Y;
                    vd.targetX = robotX * Math.cos(sense.robotAngle.getRad()) - robotY * Math.sin(sense.robotAngle.getRad()) + rse.x;
                    vd.targetY = robotX * Math.sin(sense.robotAngle.getRad()) + robotY * Math.cos(sense.robotAngle.getRad()) + rse.y;
                    */
                    targetLowStack.push(vd);
                } catch(Exception e){
                    e.printStackTrace();
                }
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            nt.addEntryListener("vis_cargo", (table,key,entry,value,flags) -> {
                try{
                    VisionData vd = new VisionData();

                    String data = value.getString(); //seqNum, dist, angle, x, y, radius
                    SmartDashboard.putString("VisionCargo", data);
                    String[] parts = data.split(",");

                    vd.distance = Double.parseDouble(parts[1]);
                    vd.angleTo = Double.parseDouble(parts[2]);
                    vd.angleOf = 0; //default 0 because angleOf is now a slope internally
                    
                    vd.timeStamp = Timer.getFPGATimestamp();
                    vd.robotAngle = sense.robotAngle.getDeg();
                    
                    //transform the camera distance vector into a field relative position
                    double camRad = vd.angleTo * Math.PI / 180;
                    double robotX = -vd.distance * Math.sin(camRad) + k.CAM_Location_X;
                    double robotY = vd.distance * Math.cos(camRad) + k.CAM_Location_Y;
                    vd.targetX = robotX * Math.cos(sense.robotAngle.getRad()) - robotY * Math.sin(sense.robotAngle.getRad()) + rse.x;
                    vd.targetY = robotX * Math.sin(sense.robotAngle.getRad()) + robotY * Math.cos(sense.robotAngle.getRad()) + rse.y;

                    cargoStack.push(vd);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        }

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

    public void run(){
        piFindCargo.setBoolean((in.enableCamera && in.actionLeft && !sense.hasCargo) || k.CAM_DebugCargo);
        piFindTargetHigh.setBoolean((in.enableCamera && in.actionLeft && sense.hasCargo) || k.CAM_DebugTargetHigh);
        piFindTargetLow.setBoolean((in.enableCamera && in.actionRight) || k.CAM_DebugTargetLow);
    }
}