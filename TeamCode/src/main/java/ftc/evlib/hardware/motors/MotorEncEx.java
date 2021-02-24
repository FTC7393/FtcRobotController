package ftc.evlib.hardware.motors;

public interface MotorEncEx extends MotorEnc {

  double getVelocity();

  void setVelocity(double ticksPerSecond);

}
