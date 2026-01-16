package frc.robot.vision;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ApriltagTarget {
    private Map<Integer, Translation3d> tagRelativePoses;

    public ApriltagTarget(Map<Integer, Translation3d> tagRelativePoses) {
        this.tagRelativePoses = tagRelativePoses;
    }

    Transform3d getHubPose(int apriltagId) {
        Translation3d relativeTagPose = tagRelativePoses.get(apriltagId);
        if (relativeTagPose != null) {
            return new Transform3d(relativeTagPose, Rotation3d.kZero);   
        }
        return null;
    }
}
