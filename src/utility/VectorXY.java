package src.utility;
public class VectorXY extends Translation2d {

  public VectorXY() {
    super();
  }

  public VectorXY(double x, double y) {
    super(x, y);
  }

  public void add(Translation2d vec) {
    x += vec.x;
    y += vec.y;
  }

  public double degrees() {
    return Math.toDegrees(Math.atan2(y, x));
  }
}


public class VectorPolarDegrees extends Translation2d {

  public VectorPolarDegrees(double r, double theta) {
    x = r * Math.cos(Math.toRadians(theta));
    y = r * Math.sin(Math.toRadians(theta));
  }
}


public class SnapshotVectorXY {
  private VectorXY vectorXY;
  private double time;

  public SnapshotVectorXY(VectorXY vectorXY, double time) {
    this.vectorXY = vectorXY;
    this.time = time;
  }

  public VectorXY getVectorXY() {
    return vectorXY;
  }

  public double getTime() {
    return time;
  }
}
