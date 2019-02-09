package frc.robot.subsystems;

public class DiskGatherer extends Component{

    public DiskGatherer() {
        
    }

    public void run() {
       if(k.D_GTH_disable) return;
       
        out.suction(in.diskGather); 
    }
}