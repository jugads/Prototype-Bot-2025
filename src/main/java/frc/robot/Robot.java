// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
//D license file in the root directory of this project.

package frc.robot;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Leds;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private Leds leds;
  private AddressableLEDBuffer buffer;
  private AddressableLED ledsObject = new AddressableLED(5);
  private Timer timer;
  private boolean increasing = true; // Tracks if brightness is increasing
private double brightness = 0;     // Current brightness (0-1 range)
private final double fadeSpeed = 0.1; // Adjust this value for fade speed
  
    

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  buffer = new AddressableLEDBuffer(138); // 138 LEDs
    ledsObject.setLength(buffer.getLength());
    ledsObject.setData(buffer);
    ledsObject.start();
    // Initialize LEDs
    leds = new Leds(ledsObject, buffer); // PWM port 9
    // Initialize timer for animation
    // Create the auto chooser

  /**
   * Creates a new auto factory for this drivetrain with the given
   * trajectory logger.
   *
   * @param trajLogger Logger for the trajectory
   * @return AutoFactory for this drivetrain
   */
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    m_robotContainer.getInput();
  }

  @Override
  public void disabledInit() {
    timer = new Timer();
    timer.start();
    // DataLogManager.start();
    // // DataLog log = DataLogManager.getLog();
    // DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().run();

    // Get the current time in seconds
    double time = timer.get();
    int length = buffer.getLength();

    for (int i = 0; i < length; i++) {
      // Spread hues across LEDs and animate over time
      int hue = (int) ((time * 100 + (i * 360.0 / length)) % 360); // Adjust time multiplier for speed
      int saturation = 255; // Full saturation
      int value = 50;      // Full brightness
      buffer.setLED(i, Color.fromHSV(hue, saturation, value));
    }

    // Push updated LED data to the strip
    ledsObject.setData(buffer);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  //   if (increasing) {
  //     brightness += fadeSpeed;
  //     if (brightness >= 1.0) { // Reached maximum brightness
  //         brightness = 1.0;
  //         increasing = false; // Start fading out
  //     }
  // } else {
  //     brightness -= fadeSpeed;
  //     if (brightness <= 0.0) { // Reached minimum brightness
  //         brightness = 0.0;
  //         increasing = true; // Start fading in
  //     }
  // }

  // // Set LED colors based on brightness
  // for (int i = 0; i < buffer.getLength(); i++) {
  //     buffer.setLED(i, new Color(brightness, 0, 0)); // Red with variable brightness
  // }
  if (drivetrain.getTV()) {
  leds.setAll(Color.kCyan);
  }
  else if (drivetrain.getTVFront()) {
    leds.setAll(Color.kRed);
  }
  else if (drivetrain.getTVRear()) {
    leds.setAll(Color.kForestGreen);
  }
  else {
    leds.setAll(Color.kBlack);
  }
  // Push updated LED data to the strip
  ledsObject.setData(buffer);

  // Optionally log the brightness for debugging
  SmartDashboard.putNumber("LED Brightness", brightness);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
