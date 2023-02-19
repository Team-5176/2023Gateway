package frc.robot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.*;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Target;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
public class Vision {
    
    // cameraName must match how the camera name set in the photonVision dashboard
    public static final PhotonCamera camera1 = new PhotonCamera("Cam1");

    public static Pose2d position;
    public static double poseAmbiguity;
    public static double timestamp; 

    //This boolean lets the position sensor fusion algorithm know when there is new vision based position information
    public static boolean hasNewTarget = false;

    // the position and orientation of camera 1 (remember, rotation values are in radians)
    private static final Pose3d cameraPose1 = new Pose3d(0.0, 0.0, 0.2, new Rotation3d(0, 0, 0)); 

    // all aprilTags on the field with their corresponding locations and tag ids
    private static Target[] staticTargets = { 
        //new Target(-2.921, 0.8382, -23, 0)
        new Target(15.513558, 1.071626, 0.462788, 1),
        new Target(15.513558, 2.748026, 0.462788, 2),
        new Target(15.513558, 4.424426, 0.462788, 3),
        new Target(16.178784, 6.749796, 0.695452, 4),
        new Target(0.36195, 6.749796, 0.695452, 5),
        new Target(1.02743, 4.424426, 0.462788, 6),
        new Target(1.02743, 2.748026, 0.462788, 7),
        new Target(1.02743, 1.071626, 0.462788, 8)
    };

    /**
     * Call this function periodically to update the vision based position estimate
     * @param heading Gyroscope based field relative heading of the robot
     */
    public static void updatePosition(double heading){
        var result = camera1.getLatestResult();
        
        if(result.hasTargets()){ //checks if target exists
            // Get the current best target.
            PhotonTrackedTarget target = result.getBestTarget();
            // Get information from target.
            int targetID = target.getFiducialId();
            poseAmbiguity = target.getPoseAmbiguity();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            
            // determine the time at which the last image was taken by reading the latency, converting to seconds, and subtracting it from the onboard clock time.
            timestamp = Timer.getFPGATimestamp() - result.getLatencyMillis() / 1000; 

            // Loop through all defined targets and fined the one with the same id as the currently tracked target
            for(int i = 0; i < staticTargets.length; i++){
                if(staticTargets[i].id == targetID){

                    hasNewTarget = true;
                    
                    //get translation from camera to target, then rotate it from camera relative orientation to field relative orientation
                    Translation3d targetTranslation = bestCameraToTarget.getTranslation().rotateBy(new Rotation3d(0, 0, -Math.toRadians(heading)));

                    // pretty sure this isn't needed for anything
                    //Rotation3d targetRotation = bestCameraToTarget.getRotation();
                    
                    //Invert the field oriented target translation, so the translation goes from target to camera, rather than from camera to target
                    double tx = targetTranslation.getX();
                    double ty = targetTranslation.getY();
                    double tz = targetTranslation.getX();
                    Translation3d inverse = new Translation3d(-tx, -ty, -tz);
                    
                    // Add the inverted field oriented target translation to the known static location of the Apriltags, which gives you the robot 3d position.
                    // This 3d position is then turned into a 2d position, because we only care about x and y.
                    // The rotation element of the pose is not important, so it is filled in with a blank rotation3d object
                    position = new Pose3d(inverse.plus(staticTargets[i].pose.getTranslation()), new Rotation3d()).toPose2d();
                    // TODO: account for camera position and orientation on the robot


                    SmartDashboard.putNumber("vision x", position.getX());
                    SmartDashboard.putNumber("vision y", position.getY());
                    
                    break;
                }
            }
        }
    }
    public static Pose2d getPositionEstimate(){
        hasNewTarget = false;
        return position;
    }
}
