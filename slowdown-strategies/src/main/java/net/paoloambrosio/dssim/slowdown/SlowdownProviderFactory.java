package net.paoloambrosio.dssim.slowdown;

import java.time.Duration;
import java.time.Instant;

public class SlowdownProviderFactory {

    public static SlowdownProvider notThreadSafe(String description) {
        final SlowdownStrategy algorithm = SlowdownStrategyFactory.fromDescription(description);
        return new SlowdownProvider() {
            @Override
            public long computeSlowdown() {
                return toMillis(algorithm.computeSlowdown(Instant.now()));
            }

            private long toMillis(Duration duration) {
                return duration.getSeconds() * 1_000 + duration.getNano() / 1_000_000;
            }
        };
    }

    public static SlowdownProvider threadSafe(String description) {
        final SlowdownProvider sp = notThreadSafe(description);
        return new SlowdownProvider() {
            @Override
            public synchronized long computeSlowdown() {
                return sp.computeSlowdown();
            }
        };
    }
}
