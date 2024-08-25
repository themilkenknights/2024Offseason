package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/*
class LoggableStdDevs implements StructSerializable, ProtobufSerializable {
  /** struct for serialization. *
  public static final LoggableStdDevsStruct struct = new LoggableStdDevsStruct();

  public LoggableStdDevs(double n1, double n2, double n3) {
    N1 = n1;
    N2 = n2;
    N3 = n3;
  }

  public LoggableStdDevs() {
  }

  public double N1, N2, N3;

  Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> toMatrix() {
    return VecBuilder.fill(N1, N2, N3);
  }
}

class LoggableStdDevsStruct implements Struct<LoggableStdDevs> {
  public String getTypeString() {
    return "struct:LoggableStdDevs";
  }

  public Class<LoggableStdDevs> getTypeClass() {
    return LoggableStdDevs.class;
  }

  @Override
  public int getSize() {
    return 3;
  }

  @Override
  public void pack(ByteBuffer bb, LoggableStdDevs devs) {
    bb.putDouble(devs.N1);
    bb.putDouble(devs.N2);
    bb.putDouble(devs.N3);
  }

  @Override
  public LoggableStdDevs unpack(ByteBuffer bb) {
    return new LoggableStdDevs(bb.getDouble(0), bb.getDouble(1), bb.getDouble(2));
  }

  @Override
  public String getSchema() {

    return "double N1;double N2;double N3";
  }
}
*/
public interface LimelightIO {
  @AutoLog
  public class LimelightIOInputs {
    Pose2d robotPoseBlue;
    double lastTimestamp;
    Pose3d[] usedTags = new Pose3d[] {};

    // LoggableStdDevs stdDevs = new LoggableStdDevs();

    double n1, n2, n3;
  }

  public default void updateInputs(LimelightIOInputs inputs) {}

  public default void setPose(Pose2d pose) {}
}
