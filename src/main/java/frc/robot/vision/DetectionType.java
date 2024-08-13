package frc.robot.vision;

public enum DetectionType { 
  DRIVER(-1),
  NOTE(1),
  APRIL_TAG(0);

  private final int pipe;

  private DetectionType(int pipe) {
    this.pipe = pipe;
  }

  public int getPipe() {
    return pipe;
  }

  public static DetectionType getType(int pipe) {
    if (pipe == 1) return NOTE;
    if (pipe == 0) return APRIL_TAG;
    return DRIVER;
  }
}
