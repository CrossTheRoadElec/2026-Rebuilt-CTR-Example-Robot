package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.function.Consumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;

public class PhotonVisionSystem {
    final AprilTagFieldLayout TagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    PhotonCamera targetCamera = new PhotonCamera("camera");
    Transform3d robotToCamera = new Transform3d(
        /* X, Y, Z */
        new Translation3d(Meters.of(0.5), Meters.of(0), Meters.of(0.5)),
        /* Roll, Pitch, Yaw */
        new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(0)));

    PhotonPoseEstimator estimator = new PhotonPoseEstimator(TagLayout, robotToCamera);
    Consumer<LoggableRobotPose> poseConsumer;

    PhotonCameraSim cameraSim = new PhotonCameraSim(targetCamera);
    VisionSystemSim visionSim = new VisionSystemSim("Camera Sim");
    LoggableRobotPose[] allPoses = new LoggableRobotPose[0];

    /* Hoot replay/autologging */
    private final HootAutoReplay autoReplay = new HootAutoReplay()
        /* We need to fetch the latest result from the photoncamera when not replaying, otherwise we need to fill the list of results when we are replaying */
        .withStructArray("CameraPoseEst/poses", LoggableRobotPose.struct, () -> allPoses, val -> allPoses = val.value);
    

    public PhotonVisionSystem(Consumer<LoggableRobotPose> poseConsumer) {
        this.poseConsumer = poseConsumer;


        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(TagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(targetCamera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, robotToCamera);

            cameraSim.enableDrawWireframe(true);
        }
    }

    public void periodic() {
        if (!Utils.isReplay()) {
            /* If this is not replay, get the hardware/simulated results from the camera */
            var allResults = targetCamera.getAllUnreadResults();
            var estimates = new ArrayList<LoggableRobotPose>();
            
            /* Process them */
            for (var result : allResults) {
                var estimate = estimator.estimateCoprocMultiTagPose(result);
                if (estimate.isEmpty()) {
                    estimate = estimator.estimateLowestAmbiguityPose(result);
                }
                estimate.ifPresent(val -> estimates.add(new LoggableRobotPose(val.estimatedPose, val.timestampSeconds)));
            }

            /* And save them so we can feed them to the drivetrain */
            allPoses = estimates.toArray(new LoggableRobotPose[0]);
        }

        /* Auto-log the poses as they come in, or pull them from the log if we're in replay */
        autoReplay.update();

        /* And process every pose we got */
        for(LoggableRobotPose pose : allPoses) {
            poseConsumer.accept(pose);
        }
    }

    public void simPeriodic(Pose2d simPose) {
        visionSim.update(simPose);
    }
    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}
