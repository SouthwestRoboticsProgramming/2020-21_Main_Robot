
public class Delay extends Trigger {
  double delay;
  Timer timer;
  public Delay(double seconds) {
    timer = new Timer();
    delay = seconds;
    timer.start();
  }
  public boolean get() {
     return timer.get() > delay;
  }
  public void whenElapsed(Command command) {
    this.whenActive(command);
  }
}
