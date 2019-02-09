package frc.robot.subsystems;

public class DiskGatherer extends Component{

    public DiskGatherer() {
        
    }

    public void run() {
       if(k.GTH_disableDisk) return;
       
        out.suction(in.diskGather); 
    }
}