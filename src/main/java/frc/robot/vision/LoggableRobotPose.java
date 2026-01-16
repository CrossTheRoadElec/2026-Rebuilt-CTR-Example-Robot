package frc.robot.vision;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;

public class LoggableRobotPose {
    public final Pose3d estimatedPose;
    public final double timestampSeconds;
    public LoggableRobotPose(Pose3d estimatedPose, double timestampSeconds) {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
    }

    public static final LoggableRobotPoseStruct struct = new LoggableRobotPoseStruct();

    
    public static class LoggableRobotPoseStruct implements Struct<LoggableRobotPose> {
        @Override
        public Class<LoggableRobotPose> getTypeClass() {
            return LoggableRobotPose.class;
        }

        @Override
        public String getTypeName() {
            return "LoggableRobotPose";
        }

        @Override
        public int getSize() {
            return Pose3d.struct.getSize() + kSizeDouble;
        }

        @Override
        public String getSchema() {
            return "Pose3d estimatedPose;double timestampSeconds";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {Pose3d.struct};
        }

        @Override
        public LoggableRobotPose unpack(ByteBuffer bb) {
            Pose3d pose = Pose3d.struct.unpack(bb);
            double time = bb.getDouble();
            return new LoggableRobotPose(pose, time);
        }

        @Override
        public void pack(ByteBuffer bb, LoggableRobotPose value) {
            Pose3d.struct.pack(bb, value.estimatedPose);
            bb.putDouble(value.timestampSeconds);
        }

        @Override
        public boolean isImmutable() {
            return true;
        }
    }
}
