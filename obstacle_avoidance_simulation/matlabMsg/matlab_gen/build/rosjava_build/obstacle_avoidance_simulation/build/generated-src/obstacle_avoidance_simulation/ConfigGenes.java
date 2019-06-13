package obstacle_avoidance_simulation;

public interface ConfigGenes extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "obstacle_avoidance_simulation/ConfigGenes";
  static final java.lang.String _DEFINITION = "# Run this in matlab to create this message: rosgenmsg(\'~/catkin_ws/src/obstacle_avoidance_simulation/matlabMsg/\')\n# New generation\nint32 Generation\n\n# New robots genes\nint32[] RobotNumber\nfloat32[] SensorActivation\nfloat32[] LinearVelocity\nfloat32[] AngularVelocity\nfloat32[] RotationTime\nfloat32[] SensorAngle\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int getGeneration();
  void setGeneration(int value);
  int[] getRobotNumber();
  void setRobotNumber(int[] value);
  float[] getSensorActivation();
  void setSensorActivation(float[] value);
  float[] getLinearVelocity();
  void setLinearVelocity(float[] value);
  float[] getAngularVelocity();
  void setAngularVelocity(float[] value);
  float[] getRotationTime();
  void setRotationTime(float[] value);
  float[] getSensorAngle();
  void setSensorAngle(float[] value);
}
