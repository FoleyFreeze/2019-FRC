package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Component {
    
    public enum ElevatorPosition {
        FLOOR, LOADING_STATION, ROCKET_1_CARGO, ROCKET_1_HATCH, ROCKET_2_CARGO, ROCKET_2_HATCH, 
        ROCKET_3_CARGO, ROCKET_3_HATCH, SHIP_CARGO, SHIP_HATCH, DONT_MOVE //note that DONT_MOVE must be the last enum element
    }
    
    public Elevator() {
        
    }

    public void run() {

        if(k.ELE_disable) return;

        String elevate;

        if (in.autoElevator){
            gotoPosition(in.elevatorTarget);
            elevate = "Auto Mode";
        } else { //in manual mode
            if (in.manualElevatorUp) {
                out.setElevatorMotor(k.ELE_MotorPwr);
                elevate = "Going Up!";
            } else if (in.manualElevatorDown) {
                out.setElevatorMotor(-k.ELE_MotorPwr);   
                elevate = "Going Down!";
            } else {
                out.setElevatorMotor(0);
                elevate = "I'm going no where...";
            }
        }
        SmartDashboard.putString("Elevator Status", elevate);
    }

    // elevator goes to set position value
    public void gotoPosition (ElevatorPosition position) {
        if(position == ElevatorPosition.DONT_MOVE){
            out.setElevatorMotor(0);
        } else {
            double setpoint = k.ELE_PositionArray[position.ordinal()] + k.ELE_OffsetHeight;
            if(in.elevatorStage && setpoint > k.ELE_StageHeight){
                setpoint = k.ELE_StageHeight;
            }
            double error = setpoint - sense.elevatorEncoder;
            double power = error*k.ELE_PositionKP;
            if(power > k.ELE_PIDLimitUp) power = k.ELE_PIDLimitUp;
            if(power < -k.ELE_PIDLimitDown) power = -k.ELE_PIDLimitDown;
            out.setElevatorMotor(power);
        }
    }
}  