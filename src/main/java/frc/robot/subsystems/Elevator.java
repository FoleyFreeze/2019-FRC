package frc.robot.subsystems;

import frc.robot.io.K;

public class Elevator extends Component {
    
    public enum ElevatorPosition {
        FLOOR, LOADING_STATION, ROCKET_1_CARGO, ROCKET_1_HATCH, ROCKET_2_CARGO, ROCKET_2_HATCH, 
        ROCKET_3_CARGO, ROCKET_3_HATCH, SHIP_CARGO, SHIP_HATCH 
        
    }
    
    public Elevator() {
        
    }

    public void run() {

        if (in.autoElevator){
            gotoPosition(in.elevatorTarget);
        } else { //in manual mode
            if (in.manualElevatorUp) {
                out.setElevatorMotor(K.ELE_MotorPwr);
            } else if (in.manualElevatorDown) {
                out.setElevatorMotor(-K.ELE_MotorPwr);   
            } else {
                out.setElevatorMotor(0);
            }
        }
    }

    public void gotoPosition (ElevatorPosition position) {
        double setpoint = K.ELE_PositionArray[position.ordinal()];

        double error = setpoint - sense.elevatorEncoder;

        out.setElevatorMotor(error*K.ELE_PositionKP);
    }
}  