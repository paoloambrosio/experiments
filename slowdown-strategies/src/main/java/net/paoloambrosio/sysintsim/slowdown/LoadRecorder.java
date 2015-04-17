package net.paoloambrosio.sysintsim.slowdown;

import java.util.stream.IntStream;

/**
 * Records requests grouping them by seconds.
 */
public interface LoadRecorder {

    /**
     * Store a request
     *
     * @param t epoch time of the request
     * @return itself for chaining
     */
    LoadRecorder store(long t);

    /**
     * Get the number of requests grouped by second
     *
     * @return request count from less recent to most recent
     */
    IntStream stream();

    /**
     * @return window size
     */
    int getWindowSize();
}
