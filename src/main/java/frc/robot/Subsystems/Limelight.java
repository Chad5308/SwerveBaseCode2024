package frc.robot.Subsystems;

import java.util.Optional;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.constants_Auto;
import frc.robot.Constants.constants_Limelight;
import frc.robot.LimelightHelpers.LimelightResults;

import java.util.function.*;

public class Limelight extends SubsystemBase{
    
public Swerve s_swerve;
public NetworkTable networkTables;
public IntegerSubscriber pipeline;
public IntegerPublisher pipelinePublisher;
public double xAng_Coral, yAng_Coral, ySpeed, xSpeed, turningSpeed, targetID_Coral, targetArea_Coral, correctionX, correctionZ, correctionT, distanceX, distanceY, yAngToRadians_Coral, xAngToRadians_Coral;
public double[] localizedPose;
public double[] botPose_targetSpace, targetPose_robotSpace;
public ProfiledPIDController thetaPIDController;
public ProfiledPIDController xPIDController, yPIDController;
public boolean autoDriveToggle;
public boolean hasTargets;
public BooleanSupplier interrupted;
public String limelight_Coral = "limelight-coral";
public String limelight_Tags = "liemlight-tags";

public LimelightResults r_limelight;

public LimelightHelpers.PoseEstimate mt2;


// public Apri fieldLayout;

// public SlewRateLimiter tLimiter, xLimiter, zLimiter;

    public Limelight(Swerve s_swerve){
        this.s_swerve = s_swerve;
        networkTables = NetworkTableInstance.getDefault().getTable("limelight");
        pipelinePublisher = networkTables.getIntegerTopic("limelight.getpipeline").publish();
        
        
        thetaPIDController = new ProfiledPIDController(constants_Limelight.thetakP, constants_Limelight.thetakI, constants_Limelight.thetakD, constants_Auto.kThetaControllerConstraints);
        xPIDController = new ProfiledPIDController(constants_Limelight.linearkP, constants_Limelight.linearkI, constants_Limelight.linearkD, constants_Auto.kLinearConstraints);
        yPIDController = new ProfiledPIDController(constants_Limelight.linearkP, constants_Limelight.linearkI, constants_Limelight.linearkD, constants_Auto.kLinearConstraints);
        setPIDControllers();

    }

    public Optional<Pose2d> getPoseFromAprilTags() {
        double[] botpose = localizedPose;
        if(botpose.length < 7 || targetID_Coral == -1) return Optional.empty();
        return Optional.of(new Pose2d(botpose[0], botpose[1], new Rotation2d(botpose[5])));
    }
  

    public void setPIDControllers()
    {
        thetaPIDController.setGoal(0);//radians
        thetaPIDController.setTolerance(Math.toRadians(0.5)); //radians

        xPIDController.setGoal(0);//meters
        xPIDController.setTolerance(0.1);//meters

        yPIDController.setGoal(1);//meters
        yPIDController.setTolerance(0.0381);//meters
    }

    // public Command autoDrive = new Command() 
    // {

        
    //     @Override
    //     public void execute()
    //     {
    //         updateDriveValues();
    //     }

    //     @Override
    //     public boolean isFinished()
    //     {
    //         return atGoal();
    //     }

    //     @Override
    //     public void end(boolean interrupted)
    //     {
    //         resetDriveValues();
    //     }
    // };


    
    
    public void updateDriveValues()
    {
        turningSpeed = thetaPIDController.calculate(xAngToRadians_Coral); /*Rads / sec */
        ySpeed = -1 * yPIDController.calculate(distanceY); /*m/sec */
    }
    public void resetDriveValues()
    {
        turningSpeed = 0;
        ySpeed = 0;
    }

    public boolean atGoal()
    {
        return yPIDController.atGoal() && thetaPIDController.atGoal();
    }


    // public Command autoDriveToggle()
    // {
    //     return Commands.runOnce(()->
    //     {
    //         autoDriveToggle = !autoDriveToggle;
    //     });
    // }

    // public Command autoDriveCheck()
    // {
    //     return Commands.waitUntil(this::atGoal);
    // }

    public Command autoDrive = new Command() {
        

        @Override
        public void initialize()
        {
            s_swerve.faceAllFoward();
        }
        
        @Override
        public void execute()
        {
            updateDriveValues();
            autoDriveToggle = true;
            s_swerve.setModuleStates(new ChassisSpeeds(ySpeed, 0, turningSpeed));
        }

        @Override
        public boolean isFinished()
        {
            return atGoal();
        }

        @Override
        public void end(boolean interrupted)
        {
            autoDriveToggle = false;
        }
    };



    @Override
    public void periodic(){
        
        
        
        // botPose_targetSpace = LimelightHelpers.getBotPose_TargetSpace(limelight_Coral);
        // targetPose_robotSpace = LimelightHelpers.getTargetPose_RobotSpace(limelight_Coral);
        // localizedPose = LimelightHelpers.getBotPose_wpiBlue(limelight_Coral);
        xAng_Coral = LimelightHelpers.getTX(limelight_Coral);
        xAngToRadians_Coral = Math.toRadians(xAng_Coral);
        yAng_Coral = LimelightHelpers.getTY(limelight_Coral);
        yAngToRadians_Coral = Math.toRadians(yAng_Coral);
        hasTargets = LimelightHelpers.getTV(limelight_Coral);
        // targetID = LimelightHelpers.getFiducialID(limelight_Coral);
        // targetArea = LimelightHelpers.getTA(limelight_Coral);
        
        distanceY = (((13 - constants_Limelight.Height_Coral) / (Math.tan(Math.toRadians(yAng_Coral+constants_Limelight.Angle_Coral)))) + constants_Limelight.DistanceForward_Coral) * 0.0254; //meters from target to center of robot
        distanceX = (Math.cos(Math.toRadians(distanceY/xAng_Coral))) * 0.0254;//meters to center of robot
        
        updateDriveValues();

        // xSpeed = -1 * xPIDController.calculate(correctionX); //m/sec


        // s_swerve.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        // s_swerve.m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);

        SmartDashboard.putNumber("Distance Horizontal", distanceX);
        SmartDashboard.putNumber("Distance Forward", distanceY);
        SmartDashboard.putNumber("Turning Speed", turningSpeed);
        // SmartDashboard.putNumber("TA Value", targetArea);
        // SmartDashboard.putNumber("X Speed", xSpeed);
        SmartDashboard.putBoolean("Has Targets", hasTargets);
        SmartDashboard.putNumber("Y Speed", ySpeed);
        SmartDashboard.putNumber("TX Value", xAng_Coral);
        SmartDashboard.putNumber("TY Value", yAng_Coral);

        SmartDashboard.putBoolean("autoDrive", autoDriveToggle);

        SmartDashboard.putBoolean("Thetha goal", thetaPIDController.atSetpoint());
        SmartDashboard.putBoolean("Linear goal", yPIDController.atSetpoint());

        // SmartDashboard.putString("Pose Estimate", mt2.pose.toString());


        // SmartDashboard.putNumber("BotPose X", botPose_targetSpace[0]);
        // SmartDashboard.putNumber("BotPose Y", botPose_targetSpace[1]);
        // SmartDashboard.putNumber("BotPose Z", botPose_targetSpace[2]);



    }
}
