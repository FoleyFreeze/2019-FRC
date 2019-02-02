package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;

public class DiskGatherer extends Component{
    
    

    public DiskGatherer() {
        
    }

    public void run() {
        out.suction(in.diskGather); 
    }
}