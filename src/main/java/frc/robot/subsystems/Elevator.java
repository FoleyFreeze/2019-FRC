package frc.robot.subsystems;

public class Elevator extends Component {
    
    public enum ElevatorPosition {
        FLOOR, LOADING_STATION, ROCKET_1_CARGO, ROCKET_1_HATCH, ROCKET_2_CARGO, ROCKET_2_HATCH, 
        ROCKET_3_CARGO, ROCKET_3_HATCH, SHIP_CARGO, SHIP_HATCH 
        
    }
    
    public Elevator() {
        
    }

    public void run() {
                if (in.manualElevatorUp) {
                    out.setElevatorMotor(k.ELE_MotorPwr);
                } else {
                    if (in.manualElevatorDown) {
                        out.setElevatorMotor(-k.ELE_MotorPwr);
                    } else {
                        if (in.autoElevator) {
                           out.setElevatorMotor(0); // temp value
                        }
                if ((!in.manualElevatorUp) && (!in.manualElevatorDown) && (!in.autoElevator)) {
                    // add in code to turn off motor
                    out.setElevatorMotor(0);
                }
            }
        }
    }
}  