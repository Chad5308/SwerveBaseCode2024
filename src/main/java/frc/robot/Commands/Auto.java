package frc.robot.Commands;



// import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.config.ModuleConfig;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.units.Distance;
// import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constants_Auto;
import frc.robot.Constants.constants_Drive;
import frc.robot.Constants.constants_Module;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Swerve;

public class Auto {
    
public Drive c_Drive;
public Swerve s_Swerve;
public Limelight s_Limelight;
public PIDController translationConstants = new PIDController(constants_Auto.kPTranslation, constants_Auto.kITranslation, constants_Auto.kDTranslation);
public PIDController rotationConstants = new PIDController(constants_Auto.kPTheta, constants_Auto.kITheta, constants_Auto.kDTheta);
// public Map map;


    public Auto(Drive c_Drive, Swerve s_Swerve, Limelight s_Limelight){
        this.c_Drive = c_Drive;
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
        // translationConstants.setTolerance(0.1);//meters
        // rotationConstants.setTolerance(10); //maybe degrees?

        AutoBuilder.configureHolonomic(
                s_Swerve::getAutoPose, 
                s_Swerve::resetOdometry, 
                s_Swerve::getRobotRelativeSpeeds, 
                s_Swerve::driveRobotRelative, 
                autoConfig, 
                s_Swerve::allianceCheck,
                s_Swerve
                );

        NamedCommands.registerCommand("FaceForward Wheels", Commands.runOnce(() -> s_Swerve.faceAllFoward()));

    }
    
    public HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(translationConstants.getP(), translationConstants.getI(), translationConstants.getD()),
        new PIDConstants(rotationConstants.getP(), rotationConstants.getI(), rotationConstants.getD()), 
        constants_Drive.kTeleDriveMaxSpeedMetersPerSecond, 
        constants_Module.moduleRadius.magnitude(), 
        new ReplanningConfig());


}
