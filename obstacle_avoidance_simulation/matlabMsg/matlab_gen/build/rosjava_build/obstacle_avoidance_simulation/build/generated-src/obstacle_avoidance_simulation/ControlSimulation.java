package obstacle_avoidance_simulation;

public interface ControlSimulation extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "obstacle_avoidance_simulation/ControlSimulation";
  static final java.lang.String _DEFINITION = "# Run this in matlab to create this message: rosgenmsg(\'~/catkin_ws/src/obstacle_avoidance_simulation/matlabMsg/\')\n# Fixed Genes\n# 0->stop 1->run 2->pause\nint32 SimulationState\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int getSimulationState();
  void setSimulationState(int value);
}
