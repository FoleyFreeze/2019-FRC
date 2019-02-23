package frc.robot.subsystems.vision;

import frc.robot.subsystems.Component;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LimitedStack;

public class Vision extends Component {

    LimitedStack<VisionData> visionTargetStack;
    LimitedStack<VisionData> cargoStack;

    public Vision() {

        visionTargetStack = new LimitedStack<>(5);
        cargoStack = new LimitedStack<>(5);

        NetworkTable nt = NetworkTableInstance.getDefault().getTable("Vision");
        nt.addEntryListener("vis_trgt", (table,key,entry,value,flags) -> {
            VisionData vd = new VisionData();

            String[] parts = value.toString().split(" ");

            vd.distance = Double.parseDouble(parts[1]);
            vd.angleTo = Double.parseDouble(parts[2]);
            vd.angleOf = Double.parseDouble(parts[3]);

            vd.timeStamp = Timer.getFPGATimestamp();
            vd.robotAngle = sense.robotAngle.getDeg();

            visionTargetStack.push(vd);

        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        nt.addEntryListener("vis_cargo", (table,key,entry,value,flags) -> {
            
            VisionData vd = new VisionData();

            String[] parts = value.toString().split(" ");

            vd.distance = Double.parseDouble(parts[1]);
            vd.angleTo = Double.parseDouble(parts[2]);
            vd.angleOf = 0;
            
            vd.timeStamp = Timer.getFPGATimestamp();
            vd.robotAngle = sense.robotAngle.getDeg();

            cargoStack.push(vd);

        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    }

    public VisionData getLastCargo(){
        return cargoStack.peek();
    }

    public VisionData getLastVisionTarget(){
        return visionTargetStack.peek();
    }
}