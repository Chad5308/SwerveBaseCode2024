package frc.robot.Subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.constants_Drive;
import frc.robot.Constants.constants_OI;

public class Swerve extends SubsystemBase{
    public final CommandXboxController opController = new CommandXboxController(constants_OI.kOPControllerPort);
    public boolean fieldOriented = false;
    public boolean hasReset = false;
    public boolean isRedAlliance;
    Optional<Alliance> alliance;
  
    
    public static Module frontLeftModule = new Module(constants_Drive.kFrontLeftTurningMotorPort, constants_Drive.kFrontLeftDriveMotorPort, constants_Drive.kFrontLeftDriveEncoderReversed, constants_Drive.kFrontLeftTurningEncoderReversed, constants_Drive.kFrontLeftDriveAbsoluteEncoderPort, constants_Drive.kBLDegrees, constants_Drive.kFrontLeftDriveAbsoluteEncoderReversed);
    public static Module frontRightModule = new Module(constants_Drive.kFrontRightTurningMotorPort, constants_Drive.kFrontRightDriveMotorPort, constants_Drive.kFrontRightDriveEncoderReversed, constants_Drive.kFrontRightTurningEncoderReversed, constants_Drive.kFrontRightDriveAbsoluteEncoderPort, constants_Drive.kBRDegrees, constants_Drive.kFrontRightDriveAbsoluteEncoderReversed);
    public static Module backLeftModule = new Module(constants_Drive.kBackLeftTurningMotorPort, constants_Drive.kBackLeftDriveMotorPort, constants_Drive.kBackLeftDriveEncoderReversed, constants_Drive.kBackLeftTurningEncoderReversed, constants_Drive.kBackLeftDriveAbsoluteEncoderPort, constants_Drive.kFLDegrees, constants_Drive.kBackLeftTurningEncoderReversed);
    public static Module backRightModule = new Module(constants_Drive.kBackRightTurningMotorPort, constants_Drive.kBackRightDriveMotorPort, constants_Drive.kBackRightDriveEncoderReversed, constants_Drive.kBackRightTurningEncoderReversed, constants_Drive.kBackRightDriveAbsoluteEncoderPort, constants_Drive.kFRDegrees, constants_Drive.kBackRightTurningEncoderReversed);
    

    public Swerve() {
        new Thread(() -> {
            try {
                Thread.sleep(500);
                zeroHeading();
            } catch (Exception e) {}}).start();    
            
            alliance = getAlliance();
        }
        
        public void resetPositions(Module FL, Module FR, Module BL, Module BR){
            FL.resetDriveEncoder();
            FR.resetDriveEncoder();
            BL.resetDriveEncoder();
            BR.resetDriveEncoder();
        }

            
    
        
    //gyro int and heading code
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    public void zeroHeading() {
        gyro.reset();
        gyro.setAngleAdjustment(0);
    }
    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }
    public Rotation2d geRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
        public boolean allianceCheck(){
        if (alliance.isPresent() && (alliance.get() == Alliance.Red)) {isRedAlliance = true;}else{isRedAlliance = false;}
        return isRedAlliance;
    }
    public Optional<Alliance> getAlliance(){
        return DriverStation.getAlliance();
    }

    //Odometer code
    public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(constants_Drive.kDriveKinematics,
    geRotation2d(), getPositions(frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()));
    
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }



    public final SwerveDriveOdometry autoOdometry = new SwerveDriveOdometry(constants_Drive.kDriveKinematics,
    geRotation2d(), getPositions(frontRightModule.getPosition(), frontLeftModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()));

    public void resetOdometry(Pose2d pose) {
        autoOdometry.resetPosition(geRotation2d(), getPositions(frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()), pose);
        // hasReset = true;
    }

    public Pose2d getAutoPose()
    {
        return autoOdometry.getPoseMeters();
    }


    public ChassisSpeeds getRobotRelativeSpeeds(){
        return constants_Drive.kDriveKinematics.toChassisSpeeds(frontLeftModule.gState(), frontRightModule.gState(), backLeftModule.gState(), backRightModule.gState());
    }
    

    //stops all modules. Called when the command isn't being ran. So when an input isn't recieved
    public void stopModules() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }
    
    public void setModuleStates(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = constants_Drive.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, constants_Drive.kPhysicalMaxSpeedMetersPerSecond);
        frontRightModule.setDesiredState(moduleStates[0]);
        frontLeftModule.setDesiredState(moduleStates[1]);
        backRightModule.setDesiredState(moduleStates[2]);
        backLeftModule.setDesiredState(moduleStates[3]);
    }
    
    public void driveRobotRelative(ChassisSpeeds speeds){
        // ChassisSpeeds.fromRobotRelativeSpeeds(speeds, geRotation2d());
        SwerveModuleState[] moduleStates = constants_Drive.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, constants_Drive.kPhysicalMaxSpeedMetersPerSecond);
        frontRightModule.setDesiredState(moduleStates[0]);
        frontLeftModule.setDesiredState(moduleStates[1]);
        backRightModule.setDesiredState(moduleStates[2]);
        backLeftModule.setDesiredState(moduleStates[3]);
    }
    
    
    //face forward method. Called once the bot is enabled
    public void faceAllFoward() {
        backRightModule.wheelFaceForward(constants_Drive.kBRDegrees);
        frontLeftModule.wheelFaceForward(constants_Drive.kFLDegrees);
        frontRightModule.wheelFaceForward(constants_Drive.kFRDegrees);
        backLeftModule.wheelFaceForward(constants_Drive.kBLDegrees);
        System.out.println("exacuted faceAll");
    }
    
    public Command resetWheels(){
        return runOnce(() -> {
                frontLeftModule.wheelFaceForward(constants_Drive.kFLDegrees);
                frontRightModule.wheelFaceForward(constants_Drive.kFRDegrees);
                backLeftModule.wheelFaceForward(constants_Drive.kBLDegrees);
                backRightModule.wheelFaceForward(constants_Drive.kBRDegrees);
    });}
        
    public Command fieldOrientedToggle(){
        return runOnce(() -> {fieldOriented = !fieldOriented;});
    }
    
    public SwerveModulePosition[] getPositions(SwerveModulePosition fl, SwerveModulePosition fr, SwerveModulePosition bl, SwerveModulePosition br){
        return new SwerveModulePosition[]{
            fl, fr, bl, br
        };
    }


    @Override
    public void periodic() {
            odometer.update(geRotation2d(), getPositions(
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition())
            );

            autoOdometry.update(geRotation2d(), getPositions(
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition())
            );



        
        
        //multiple debugging values are listed here. Names are self explanitory
        
                //Odometer and other gyro values
               SmartDashboard.putString("Robot Location", getAutoPose().getTranslation().toString());
               SmartDashboard.putNumber("Robot Heading", getHeading());
              
               //AE Degrees Reading
            //     SmartDashboard.putNumber("Back Left AE Value", backLeftModule.getAbsoluteEncoderDeg(constants_Drive.kBLDegrees));
            //     SmartDashboard.putNumber("Back Right AE Value", backRightModule.getAbsoluteEncoderDeg(constants_Drive.kBRDegrees));
            //     SmartDashboard.putNumber("Front Left AE Value", frontLeftModule.getAbsoluteEncoderDeg(constants_Drive.kFLDegrees));
            //     SmartDashboard.putNumber("Front Right AE Value", frontRightModule.getAbsoluteEncoderDeg(constants_Drive.kFRDegrees));
            // //   //RE Degrees Reading
            //     SmartDashboard.putNumber("Back left RE Value", backLeftModule.getSteerPosition());
            //     SmartDashboard.putNumber("Back Right RE Value", backRightModule.getSteerPosition());
            //     SmartDashboard.putNumber("Front left RE Value", frontLeftModule.getSteerPosition());
            //     SmartDashboard.putNumber("Front Right RE Value", frontRightModule.getSteerPosition());
            //  //RE Distance Reading
            //    SmartDashboard.putNumber("Front Left Drive Position", frontLeftModule.getDrivePosition());
            //    SmartDashboard.putNumber("Front Right Drive Position", frontRightModule.getDrivePosition());
            //    SmartDashboard.putNumber("Back Left Drive Position", backLeftModule.getDrivePosition());
            //    SmartDashboard.putNumber("Back Right Drive Position", backRightModule.getDrivePosition());
            

            //  flModPos = new SwerveModulePosition(frontLeftModule.getPositionMeters(), frontLeftModule.gState().angle);
            //  frModPos = new SwerveModulePosition(frontRightModule.getPositionMeters(), frontRightModule.gState().angle);
            //  blModPos = new SwerveModulePosition(backLeftModule.getPositionMeters(), backLeftModule.gState().angle);
            //  brModPos = new SwerveModulePosition(backRightModule.getPositionMeters(), backRightModule.gState().angle);
        }
}
