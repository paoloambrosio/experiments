package net.paoloambrosio.sysintsim.slowdown;

import java.lang.annotation.ElementType;
import java.lang.annotation.Target;
import java.time.Duration;
import java.time.Instant;

public interface SlowdownStrategy {

    /**
     * Annotates builder method with a name.
     */
    @Target(ElementType.METHOD)
    @interface Builder {
        String value();
    }

    /**
     * Computes the response time for the current request
     *
     * @param t time of the request
     * @return time it should take
     */
    Duration computeSlowdown(Instant t);
}
