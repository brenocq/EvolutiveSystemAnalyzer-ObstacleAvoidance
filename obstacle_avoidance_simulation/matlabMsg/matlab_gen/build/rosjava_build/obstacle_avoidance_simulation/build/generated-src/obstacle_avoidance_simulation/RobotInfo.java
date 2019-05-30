package obstacle_avoidance_simulation;

public interface RobotInfo extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "obstacle_avoidance_simulation/RobotInfo";
  static final java.lang.String _DEFINITION = "# Run this in matlab to create this message: rosgenmsg(\'~/catkin_ws/src/obstacle_avoidance_simulation/matlabMsg/\')\n# Robot Information\nint32[] RobotNumber\nint32[] Fitness\nint32 Generation\n# Chromosome Information\nfloat32[] SensorActivation\nfloat32[] LinearVelocity\nfloat32[] AngularVelocity\nfloat32[] RotationTime\nfloat32[] SensorAngle\nbool[] Mutation\n# {[5],[5],...,[5]} (each robot uses 5 indexes(one for gene))\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int[] getRobotNumber();
  void setRobotNumber(int[] value);
  int[] getFitness();
  void setFitness(int[] value);
  int getGeneration();
  void setGeneration(int value);
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
  boolean[] getMutation();
  void setMutation(boolean[] value);
}
