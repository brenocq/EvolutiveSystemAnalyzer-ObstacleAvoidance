package turtlebot3_test_obst_avoid;

public interface RobotInfo extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "turtlebot3_test_obst_avoid/RobotInfo";
  static final java.lang.String _DEFINITION = "# Robot Information\n# rosgenmsg(\'/home/breno/catkin_ws/src/turtlebot3_test_obst_avoid/matlabMsg/\')\nint32[] RobotNumber\nint32[] Fitness\nint32 Generation\n# Chromosome Information\nfloat32[] SensorActivation\nfloat32[] LinearVelocity\nfloat32[] AngularVelocity\nfloat32[] RotationTime\nfloat32[] SensorAngle\nbool[] Mutation\n# {[5],[5],...,[5]} (each robot uses 5 indexes(one for gene))\n";
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
