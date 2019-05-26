package obstacle_avoidance_simulation;

public interface EvolutiveSystemConfiguration extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "obstacle_avoidance_simulation/EvolutiveSystemConfiguration";
  static final java.lang.String _DEFINITION = "# Run this in matlab to create this message: rosgenmsg(\'/home/breno/catkin_ws/src/obstacle_avoidance_simulation/matlabMsg/\')\n# Fixed Genes\nfloat32 SensorActivation\nfloat32 LinearVelocity\nfloat32 AngularVelocity\nfloat32 RotationTime\nfloat32 SensorAngle\n\n# Types of Pressure\nbool BackMutationPrevention\nint32 Predation\n\n# Types of Diversity\nfloat32 Mutation\nfloat32 Neutralization\nbool Crossing\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  float getSensorActivation();
  void setSensorActivation(float value);
  float getLinearVelocity();
  void setLinearVelocity(float value);
  float getAngularVelocity();
  void setAngularVelocity(float value);
  float getRotationTime();
  void setRotationTime(float value);
  float getSensorAngle();
  void setSensorAngle(float value);
  boolean getBackMutationPrevention();
  void setBackMutationPrevention(boolean value);
  int getPredation();
  void setPredation(int value);
  float getMutation();
  void setMutation(float value);
  float getNeutralization();
  void setNeutralization(float value);
  boolean getCrossing();
  void setCrossing(boolean value);
}
