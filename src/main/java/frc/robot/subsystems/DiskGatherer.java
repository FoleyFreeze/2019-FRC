package frc.robot.subsystems;

public class DiskGatherer extends Component{

    public DiskGatherer() {
        
    }

    public void run() {
        out.suction(in.diskGather); 
    }
}