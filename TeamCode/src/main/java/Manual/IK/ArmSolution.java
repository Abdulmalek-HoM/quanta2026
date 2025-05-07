package Manual.IK;

public class ArmSolution {
    public double theta1;  // Base angle (degrees)
    public double theta2;  // Tilt angle (degrees)
    public double theta3;  // Wrist angle (degrees)
    public double slideExtension;  // mm

    public ArmSolution(double t1, double t2, double t3, double slide) {
        this.theta1 = t1;
        this.theta2 = t2;
        this.theta3 = t3;
        this.slideExtension = slide;
    }
}