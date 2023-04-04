package frc.robot.utilities;
import java.util.concurrent.atomic.AtomicBoolean;

public class RunOnce {

    private final AtomicBoolean finished = new AtomicBoolean();

    public void run(Runnable task) {
        if (finished.get()) return;
        if (finished.compareAndSet(false, true)) {
            task.run();
        }
    }
}
