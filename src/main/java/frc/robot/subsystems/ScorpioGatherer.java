package frc.robot.subsystems;

public class ScorpioGatherer extends ArmGatherer {

    public ScorpioGatherer(){

    }

    @Override
    public void run(){
        if(k.GTH_DisableGather) return;

        if(in.controlBoard.jogUp){
            out.setGatherArmPosition(0.5);
        } else if(in.controlBoard.jogDown){
            out.setGatherArmPosition(-0.5);
        } else {
            out.setGatherArmPosition(0);
        }

        if(in.gather) out.setGatherWheels(0.5);
        else if(in.shoot) out.setGatherWheels(-0.5);
        else out.setGatherWheels(0);
    }

}