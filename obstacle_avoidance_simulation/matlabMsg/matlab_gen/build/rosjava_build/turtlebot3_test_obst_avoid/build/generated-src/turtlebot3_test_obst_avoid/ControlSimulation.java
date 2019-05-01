package turtlebot3_test_obst_avoid;

public interface ControlSimulation extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "turtlebot3_test_obst_avoid/ControlSimulation";
  static final java.lang.String _DEFINITION = "# Fixed Genes\n# 0->stop 1->run 2->pause\nint32 SimulationState\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int getSimulationState();
  void setSimulationState(int value);
}
