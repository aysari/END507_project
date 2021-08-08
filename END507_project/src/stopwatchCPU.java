import java.lang.management.ThreadMXBean;
import java.lang.management.ManagementFactory;

public class stopwatchCPU{
    private static final double NANOSECONDS_PER_SECOND = 1000000000;

    private ThreadMXBean threadTimer;
    private long start;
            
    /**
     * Initializes a new stopwatch.
     */
    
    public stopwatchCPU() {  
        threadTimer = ManagementFactory.getThreadMXBean();
        start = threadTimer.getCurrentThreadCpuTime();
    }   
        
    /**
     * Returns the elapsed CPU time (in seconds) since the stopwatch was created.
     *
     * @return elapsed CPU time (in seconds) since the stopwatch was created
     */
    
    public double elapsedTime() {
        long now = threadTimer.getCurrentThreadCpuTime();
        return (now - start) / NANOSECONDS_PER_SECOND;
    }
}