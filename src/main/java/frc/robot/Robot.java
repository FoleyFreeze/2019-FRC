/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.io.Display;
import frc.robot.subsystems.Component;

public class Robot extends TimedRobot {
  
  @Override
  public void robotInit() {
    Component.initAll();
    Display.run();
  }

  @Override
  public void robotPeriodic() {
    Component.runAll();
  }

  @Override
  public void autonomousInit() {
    Component.sense.init();// zero the navx, but only at the beginning of a match
    Component.sense.isDisabled = false;
    Component.sense.isAuto = true;
    Component.sense.isTeleop = false;
  }

  @Override
  public void teleopInit() {
    Component.sense.isDisabled = false;
    Component.sense.isAuto = false;
    Component.sense.isTeleop = true;
  }

  @Override
  public void disabledInit() {
    Component.sense.isDisabled = true;
    Component.sense.isAuto = false;
    Component.sense.isTeleop = false;
  }

}
