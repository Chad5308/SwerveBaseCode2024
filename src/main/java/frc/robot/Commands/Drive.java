package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.constants_OI;
import frc.robot.Constants.constants_Drive;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Swerve;


public class Drive extends Command{
    

    
    private final Swerve s_Swerve;
    private final Limelight s_limelight;
    private final Robot robot;
    public final CommandXboxController opController;
    // public final CommandJoystick leftStick;
    // public final CommandJoystick rightStick;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean fieldOriented=false;
     public double ySpeed, xSpeed, turningSpeed;
     public double ll_zSpeed, ll_xSpeed, ll_turningSpeed;
    public ChassisSpeeds chassisSpeeds;

    



    // public DriveCommand(s_Swerve s_Swerve, CommandXboxController opController, CommandJoystick leftStick, CommandJoystick rightStick) {
        public Drive(Swerve s_Swerve, CommandXboxController opController, Limelight s_limelight, Robot robot) {

                this.s_Swerve = s_Swerve;
                this.s_limelight = s_limelight;
                this.robot = robot;
                this.xLimiter = new SlewRateLimiter(constants_Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.yLimiter = new SlewRateLimiter(constants_Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.turningLimiter = new SlewRateLimiter(constants_Drive.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
                addRequirements(s_Swerve);
                this.opController = opController;
                // this.leftStick = leftStick;
                // this.rightStick = rightStick;
    }


    @Override
    public void initialize() {
     s_Swerve.faceAllFoward();
    }

 


    @Override
    public void execute() {
      
        // xSpeed = IsJoyStick? -leftStick.getX(): -opController.getLeftX();
        // ySpeed = IsJoyStick? -leftStick.getY(): -opController.getLeftY();
        // turningSpeed = IsJoyStick? -rightStick.getX(): -opController.getRightX();
        xSpeed = -opController.getLeftX();
        ySpeed = -opController.getLeftY();
        turningSpeed = -opController.getRightX();
        fieldOriented = s_Swerve.fieldOriented;


        
        SmartDashboard.putBoolean("fieldOriented", fieldOriented);


        xSpeed = Math.abs(xSpeed) > constants_OI.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > constants_OI.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > constants_OI.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * constants_Drive.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * constants_Drive.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * constants_Drive.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        
        drive();
    }
    
    public void drive()
    {
        ChassisSpeeds chassisSpeeds;
        
        if(!robot.isAutonomous())
        {
            if(fieldOriented)
            {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, turningSpeed, s_Swerve.geRotation2d());
            }else
            {
                chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);
            }
            s_Swerve.setModuleStates(chassisSpeeds);        
        }
        

    }


    @Override
    public void end(boolean interrupted) {
        s_Swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }



}
