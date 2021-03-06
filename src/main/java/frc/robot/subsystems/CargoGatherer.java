package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.ControlBoard.NearFarCargo;

public class CargoGatherer extends Component{

    public CargoGatherer() {
        
    }
    
    public void run() {
        String gatherStatus;
        if(k.GTH_DisableCargo) return;
        
        // conditions for gathering
        if(in.gatherCargo){
            out.setGatherMotor(k.GTH_CargoIntakeSpeed, -k.GTH_CargoIntakeSpeed);
            gatherStatus = "Gathering";
        }
        //  releases cargo
        else if(in.releaseCargo){
                if (in.controlBoard.nearFarCargo == NearFarCargo.CARGO) {
                    out.setGatherMotor(-k.GTH_CargoShootSpeedFast, k.GTH_CargoShootSpeedSlow);
                } else {
                    out.setGatherMotor(-k.GTH_CargoShootSpeedFast, k.GTH_CargoShootSpeedSlow);
                }

            gatherStatus = "Releasing";
        }
        else if(in.gatherHatch){
            out.setGatherMotor(-k.GTH_HatchIntakeSpeed, k.GTH_HatchIntakeSpeed);
            gatherStatus = "Gathering";
        }
        //  releases hatch
        else if(in.releaseHatch){
            out.setGatherMotor(k.GTH_HatchShootSpeedFast, -k.GTH_HatchShootSpeedSlow);
            gatherStatus = "Releasing";
        }
        else if(sense.hasCargo){
            gatherStatus = "Hold";
            out.setGatherMotor(k.GTH_CargoHoldSpeed, -k.GTH_CargoHoldSpeed);
        }
        else if(sense.hasHatch){
            gatherStatus = "Hold";
            out.setGatherMotor(-k.GTH_HatchHoldSpeed, k.GTH_HatchHoldSpeed);
        }
        // stop moving
        else {
            out.setGatherMotor(0,0); 
            gatherStatus = "Stopped";
        }
        SmartDashboard.putString("Gather Status", gatherStatus);
    }

} 