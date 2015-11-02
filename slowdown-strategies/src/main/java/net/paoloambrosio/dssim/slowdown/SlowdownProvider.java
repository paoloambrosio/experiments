package net.paoloambrosio.dssim.slowdown;

public interface SlowdownProvider {

    /**
     * Computes the response time for the current request
     *
     * @return milliseconds it should take
     */
    long computeSlowdown();
}
