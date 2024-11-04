// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Auto;
import frc.robot.Commands.Drive;
import frc.robot.Constants.constants_OI;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Swerve;


/**
 * This class is where the bulk
 *  of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController opController = new CommandXboxController(constants_OI.kOPControllerPort);
  // private final CommandJoystick leftStick = new CommandJoystick(OIConstants.kLeftStickPort);
  // private final CommandJoystick rightStick = new CommandJoystick(OIConstants.kRightStickPort);

  public static Robot robot = new Robot();
  public Swerve s_Swerve = new Swerve();
  public LimelightHelpers h_Limelight = new LimelightHelpers();
  public Limelight s_Limelight = new Limelight(s_Swerve);
  public Drive c_Drive = new Drive(s_Swerve, opController, s_Limelight, robot);
  public Auto c_Auto = new Auto(c_Drive, s_Swerve, s_Limelight);

  private SendableChooser<Command> autoChooser;


 
  public RobotContainer() 
  {
    autoChooser = AutoBuilder.buildAutoChooser();
    s_Swerve.setDefaultCommand(c_Drive);
    configureBindings();
    configureAuto();

    SmartDashboard.putData("Auto Chooser", autoChooser);   
  }

  public void configureAuto()
  {
    autoChooser.addOption("AutoDrive", limelightTestAuto());
  }

  public SequentialCommandGroup limelightTestAuto()
  {
    return new SequentialCommandGroup(new PathPlannerAuto("Start Auto").andThen(s_Limelight.autoDrive).andThen(new PathPlannerAuto("Return Auto")));
  }



  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  private void configureBindings() {
    //Drive Controls
    opController.povRight().toggleOnTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
    opController.povLeft().toggleOnTrue(s_Swerve.fieldOrientedToggle());
    opController.button(7).onTrue(s_Swerve.resetWheels()); //window looking button
  }
}
