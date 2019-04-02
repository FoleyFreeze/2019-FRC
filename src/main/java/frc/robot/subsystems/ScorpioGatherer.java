package frc.robot.subsystems;

public class ScorpioGatherer extends ArmGatherer {

    public ScorpioGatherer(){

    }

    @Override
    public void run(){
        if(k.GTH_DisableGather) return;

        if(!in.autoNotManualMode){
            if(in.controlBoard.jogUp){
                out.setGatherArm(0.5);
            } else if(in.controlBoard.jogDown){
                out.setGatherArm(-0.5);
            } else {
                out.setGatherArm(0);
            }

            if(in.gather) out.setGatherWheels(0.5);
            else if(in.shoot) out.setGatherWheels(-0.5);
            else out.setGatherWheels(0);
        }
        else { //automatic mode
            out.setGatherArm(0);
            out.setGatherWheels(0);
        }
    }

}