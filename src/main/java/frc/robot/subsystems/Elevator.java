package frc.robot.subsystems;

public class Elevator extends Component {
    
    public enum ElevatorPosition {
        FLOOR, LOADING_STATION, ROCKET_1_CARGO, ROCKET_1_HATCH, ROCKET_2_CARGO, ROCKET_2_HATCH, 
        ROCKET_3_CARGO, ROCKET_3_HATCH, SHIP_CARGO, SHIP_HATCH, DONT_MOVE //note that DONT_MOVE must be the last enum element
    }
    
    public Elevator() {
        
    }

    public void run() {

        if(k.ELE_disable) return;

        if (in.autoElevator){
            gotoPosition(in.elevatorTarget);
        } else { //in manual mode
            if (in.manualElevatorUp) {
                out.setElevatorMotor(k.ELE_MotorPwr);
            } else if (in.manualElevatorDown) {
                out.setElevatorMotor(-k.ELE_MotorPwr);   
            } else {
                out.setElevatorMotor(0);
            }
        }
    }

    public void gotoPosition (ElevatorPosition position) {
        if(position == ElevatorPosition.DONT_MOVE){
            out.setElevatorMotor(0);
        } else {
            double setpoint = k.ELE_PositionArray[position.ordinal()];
            double error = setpoint - sense.elevatorEncoder;
            out.setElevatorMotor(error*k.ELE_PositionKP);
        }
    }
}  