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

    LimitedStack<VisionData> visionTargetStack;
    LimitedStack<VisionData> cargoStack;

    NetworkTable piCommands;
    NetworkTableEntry piDebugMode;
    NetworkTableEntry piFindCargo;
    NetworkTableEntry piFindVisionTarget;

    public Vision() {
        piCommands = NetworkTableInstance.getDefault().getTable("PiControl");
        piDebugMode = piCommands.getEntry("DebugMode");
        piFindCargo = piCommands.getEntry("FindCargo");
        piFindVisionTarget = piCommands.getEntry("FindVisionTarget");

        visionTargetStack = new LimitedStack<>(5);
        cargoStack = new LimitedStack<>(5);
        if(!k.CAM_Disabled){
            NetworkTable nt = NetworkTableInstance.getDefault().getTable("Vision");
            nt.addEntryListener("vis_trgt", (table,key,entry,value,flags) -> {
                try{
                    VisionData vd = new VisionData();
                    String data = value.getString(); //seqNum, dist, angleOf, angleTo, etc?
                    SmartDashboard.putString("VisionTarget", data);
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

                    visionTargetStack.push(vd);
                } catch(Exception e){
                    e.printStackTrace();
                }
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            nt.addEntryListener("vis_cargo", (table,key,entry,value,flags) -> {
                try{
                    VisionData vd = new VisionData();

                    String data = value.getString(); //seqNum, dist, angle, x, y, w, h
                    SmartDashboard.putString("VisionCargo", data);
                    String[] parts = data.split(",");

                    vd.distance = Double.parseDouble(parts[1]);
                    vd.angleTo = Double.parseDouble(parts[2]);
                    vd.angleOf = 90;
                    
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

    public boolean goodVisionTarget(){
        //TODO: eventually check the image vs the expected location of the target
        VisionData vd = visionTargetStack.peek();
        return vd != null && Timer.getFPGATimestamp() - vd.timeStamp < k.CAM_ExpireTime; 
    }

    public VisionData getLastCargo(){
        return cargoStack.peek();
    }

    public VisionData getLastVisionTarget(){
        return visionTargetStack.peek();
    }

    public void run(){
        piDebugMode.setBoolean(k.CAM_DebugMode);
        piFindCargo.setBoolean(in.enableCamera && in.actionCargo && !sense.hasCargo);
        piFindVisionTarget.setBoolean(in.enableCamera && (in.actionHatch || in.actionCargo && sense.hasCargo));
    }
}