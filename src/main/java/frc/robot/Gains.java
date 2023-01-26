package frc.robot;

public class Gains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kMinOutput;
	public final double kMaxOutput;


	public Gains(double _kP, double _kI, double _kD, double _kMinOutput, double _kMaxOutput){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kMinOutput = _kMinOutput;
		kMaxOutput = _kMaxOutput;
	}

	@Override
	public String toString() {
		return "P: " + kP + " I: " + kI + " D: " + kD;
	}
}