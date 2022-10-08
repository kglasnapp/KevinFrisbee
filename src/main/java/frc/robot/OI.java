package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Config.RobotType;
import frc.robot.commands.TestCmd;
import frc.robot.commands.ZeroYawAndPosition;
import frc.robot.subsystems.Joysticks;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.KevinShooterCmd;
import frc.robot.commands.DriveStraight.DriveMode;
import frc.robot.commands.KevinShooterCmd.ShooterMode;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  static enum Action {
    PRESSED, RELEASED
  }

  private ArrayList<ButtonHandler> buttons = new ArrayList<ButtonHandler>();

  void setShooterPOV(Joystick joy, int angle, int speed) {
    // new POVButton(joy, angle).whenPressed(new
    // ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, speed));
  }

  OI() {
    SmartDashboard.putData("TestTask", new TestCmd());
    if (Config.robotType == RobotType.Kevin) {
      KevinOI();
    }
  }

  final public int defaultShot = 6;

  void KevinOI() {
    //Joystick driver = Robot.joysticks.getDriverPad();
    Joystick op = Robot.joysticks.getOperatorPad();
    new ButtonHandler(op, 4, Action.PRESSED, new ZeroYawAndPosition(), "Zero Yaw");
    new ButtonHandler(op, 6, Action.PRESSED, new KevinShooterCmd(ShooterMode.SHOOT), "Shoot");
  
    SmartDashboard.putData("For 3'", new DriveStraight(DriveMode.RELATIVE_INCHES,
        36, .3, 5));
    SmartDashboard.putData("Back 3'", new DriveStraight(DriveMode.RELATIVE_INCHES, -36, .3, 5, true));
    SmartDashboard.putData("For 1'", new DriveStraight(DriveMode.RELATIVE_INCHES,
        12, .5, 5, true));
    SmartDashboard.putData("Back 1'", new DriveStraight(DriveMode.RELATIVE_INCHES, -12, .5, 5, true));
    SmartDashboard.putData("For 3'|.3|30D", new DriveStraight(DriveMode.RELATIVE_INCHES, false, 30, 36, .3, 5));
    SmartDashboard.putData("For 1'|.3|30D", new DriveStraight(DriveMode.RELATIVE_INCHES, false, 30, 12, .3, 5));
    SmartDashboard.putData("Reverse Shooter", new KevinShooterCmd(ShooterMode.REVERSE_SHOOTER));
    
  }

  // Left Joy actions
  public boolean driveStraightPressed() {
    if ((Robot.driveJoy)) {
      return Joysticks.leftJoy.getTriggerPressed();
    }
    if (Joysticks.driver != null)
      return Joysticks.driver.getRawButtonPressed(5);
    if (Joysticks.leftJoy == null)
      return Joysticks.operator.getRawButtonPressed(5);
    return Joysticks.leftJoy.getTriggerPressed();
  }

  public boolean driveStraightReleased() {
    if (Robot.driveJoy) {
      return Joysticks.leftJoy.getTriggerReleased();
    }
    if (Joysticks.driver != null)
      return Joysticks.driver.getRawButtonReleased(5);
    if (Joysticks.leftJoy == null)
      return Joysticks.operator.getRawButtonReleased(5);
    return Joysticks.leftJoy.getTriggerReleased();
  }

  public double driveStraightSpeed() {
    if (Robot.driveJoy) {
      return (Joysticks.rightJoy.getY() + Joysticks.leftJoy.getY()) / 2;
    }
    if (Joysticks.driver != null) {
      return (Joysticks.driver.getRawAxis(5) + Joysticks.driver.getRawAxis(1)) / 2;
    }
    if (Joysticks.leftJoy == null)
      return (Joysticks.operator.getRawAxis(5) + Joysticks.operator.getRawAxis(1)) / 2;
    return (Joysticks.rightJoy.getY() + Joysticks.leftJoy.getY()) / 2;
  }

  public double leftJoySpeed() {
    if (Robot.driveJoy) {
      return -Joysticks.leftJoy.getY();
    }
    if (Joysticks.driver != null) {
      return -Joysticks.driver.getRawAxis(1);
    }
    if (Joysticks.rightJoy == null)
      return -Joysticks.operator.getRawAxis(1);
    return -Joysticks.leftJoy.getY();
  }

  // Right Joy Actions

  public double rightJoySpeed() {
    if (Robot.driveJoy) {
      return -Joysticks.rightJoy.getY();
    }
    if (Joysticks.driver != null) {
      return -Joysticks.driver.getRawAxis(5);
    }
    if (Joysticks.rightJoy == null)
      return -Joysticks.operator.getRawAxis(5);
    return -Joysticks.rightJoy.getY();
  }

  public boolean ballTrackActive() {
    return Joysticks.rightJoy.getRawButton(2);
  }

  // Operator actions
  public boolean clearStickyAndLogCurrents() {
    return Joysticks.operator.getRawButtonPressed(9);
  }

  public boolean turboMode() {
    if (Joysticks.leftJoy == null)
      return false;
    return Joysticks.leftJoy.getPOV() == 0;
  }

  class ButtonHandler {
    int port;
    Joystick joystick;
    int buttonNumber;
    Action act;
    String name;

    private ButtonHandler(Joystick joystick, int buttonNumber, Action act, CommandBase cmd, String name) {
      if (joystick == null)
        return;
      this.joystick = joystick;
      this.buttonNumber = buttonNumber;
      this.act = act;
      this.name = name;
      port = joystick.getPort();
      buttons.add(this);
      JoystickButton button = new JoystickButton(joystick, buttonNumber);
      if (act == Action.PRESSED)
        button.whenPressed(cmd);
      if (act == Action.RELEASED)
        button.whenReleased(cmd);
      // todo took out button.close();
    }

    String getData() {
      return "Button:" + name + " Port:" + port + " Button:" + buttonNumber + " Action:" + act;
    }
  }
}