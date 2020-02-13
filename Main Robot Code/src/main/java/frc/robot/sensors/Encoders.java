
public class Encoders {

    private R<Doub> position;

    public Encoders(double THETA_INIT) {
        
        
        position = new R<Doub>({0,0});
    }
    
    public double radius() {
        return K.TRACK / 2 * (left_prime() + right_prime()) / (left_prime() - right_prime());
    }
    private double left_prime() {
        return Robot.driveTrainSubsystem.getLeftVelocity();
    }
    private double right_prime() {
        return Robot.driveTrainSubsystem.getRightVelocity();
    }
    private double left() {
        return Robot.driveTrainSubsystem.getLeftDriveFeet();
    }
    private double right() {
        return Robot.driveTrainSubsystem.getRightDriveFeet();
    }
    public double omega() {
        return (right_prime() - left_prime()) / K.TRACK;
    }
    public double theta() {
        return (right() - left()) / K.TRACK;
    }
    public double x_prime() {
        return radius() * omega() * Math.cos(theta());
    }
    public double y_prime() {
        return radius() * omega() * Math.sin(theta());
    }
    public R<Doub> v() {
        return new R<Doub>(x_prime(), y_prime());
    }
    public double x() {
        return position.get(0);
    }
    public double y() {
        return position.get(1);
    }
    public R<Doub> pos() {
        return position;
    }
    public void step() {
        position = position.add( v().mul(new R<Doub>(K.TIME_STEP)) ); // only 
    }


}
