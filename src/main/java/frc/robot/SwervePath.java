package frc.robot;

import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;

public class SwervePath {
    public List<State> trajectoryPath;
    public double beginning;
    public double end;
    public double deadZone;
    public int idx;

    public double prevTime;
    double startTime;

    public SwervePath(double initHeading, double endHeading, double deadzone) {
        //trajectoryPath = Robot.path.getStates();
        trajectoryPath = DriveSubsystem.squarePath.getStates();

        TrajectoryConfig  tc = new TrajectoryConfig(.1, .5);



        //trajectoryPath = TrajectoryGenerator.generateTrajectory(, null, null, tc)
        
        end = endHeading;
        beginning = initHeading;
        deadZone = deadzone;
        idx = 1;


    }


    public ChassisSpeeds getSpeeds(Pose2d currentPose) {

      State desiredState = this.trajectoryPath.get(idx);
      

      Pose2d desiredPose = this.trajectoryPath.get(idx).poseMeters;        
      double desiredVelocity = Math.abs(desiredState.velocityMetersPerSecond);
      //double deltaTime = trajectoryPath.get(idx).timeSeconds - trajectoryPath.get(idx-1).timeSeconds;

      Transform2d distance = desiredPose.minus(currentPose);
  
      if(Math.abs(distance.getX()) <= deadZone && Math.abs(distance.getY()) <= deadZone){
        if(idx < trajectoryPath.size()-1) {
          idx++;
          //System.out.println(idx);
        } else {
          return ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, currentPose.getRotation());
        }
      }
      // Handy dandy trigonometry
      // Why u no work, me use mathematics
      /*double angle = Math.atan(distance.getX()/distance.getY());

      double xSpeed = desiredVelocity*Math.sin(angle);
      double ySpeed = desiredVelocity*Math.cos(angle);
      double rotation = distance.getRotation().getRadians();

      System.out.println(desiredVelocity);
      System.out.println(angle);
      System.out.println(xSpeed);
      System.out.println(ySpeed);*/
      
      double magnitude = Math.sqrt(Math.pow(distance.getX(), 2) + Math.pow(distance.getY(), 2));

      double xSpeed = distance.getX() / magnitude * desiredVelocity;
      double ySpeed = distance.getY() / magnitude * desiredVelocity;
      
      
      double rotation = distance.getRotation().getRadians();

      // xSpeed *= deltaTime;
      // ySpeed *= deltaTime;
      // rotation *= deltaTime;
  
      ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 0, currentPose.getRotation());

      return desiredSpeeds;
    }



    }

    




/*

*/
