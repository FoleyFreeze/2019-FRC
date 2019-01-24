package frc.robot.subsystems;

public class Elevator extends Component {
    
    
    
    public Elevator() {
        
    }

    public void run() {
                if (in.manualElevatorUp) {
                    // add in code to turn on motor and go up
                } else {
                    if (in.manualElevatorDown) {
                        // add in code to turn on motor and go down
                    } else {
                        if (in.elevatorTarget) {
                            // add in code to move elevator to specific location
                        }
                if ((!in.manualElevatorUp) && (!in.manualElevatorDown) && (!in.elevatorTarget)) {
                    // add in code to turn off motor
                }
            }
        }
    }
}  


                