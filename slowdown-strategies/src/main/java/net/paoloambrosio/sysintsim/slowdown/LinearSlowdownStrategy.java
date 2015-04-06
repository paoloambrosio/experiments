package net.paoloambrosio.sysintsim.slowdown;

import java.time.Duration;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneId;

public final class LinearSlowdownStrategy implements SlowdownStrategy {

    private final LoadRecorder loadRecorder;
    private final Duration maxSlowdown;

    public LinearSlowdownStrategy(Duration maxSlowdown, LoadRecorder loadRecorder) {
        this.maxSlowdown = maxSlowdown;
        this.loadRecorder = loadRecorder;
    }

    public Duration getMaxSlowdown() {
        return maxSlowdown;
    }

    public LoadRecorder getLoadRecorder() {
        return loadRecorder;
    }

    @Override
    public Duration computeSlowdown(Instant t) {
        loadRecorder.store(t.toEpochMilli() / 1000);
        double average = loadRecorder.stream().average().orElse(0.0);
        return Duration.ofMillis((long) average);
    }

    /**
     * Builds a {@link net.paoloambrosio.sysintsim.slowdown.LinearSlowdownStrategy} from a max duration and
     * a window size parameters.
     *
     * @param parameters max duration (optional, defaults to 60s), window size (optional, defaults to 60s)
     * @return strategy
     */
    @Builder("linear")
    public static LinearSlowdownStrategy build(String... parameters) {
        if (parameters.length > 2) {
            String message = String.format("At most two parameters expected (was: %d)", parameters.length);
            throw new IllegalArgumentException(message);
        }
        Duration maxSlowdown = extractMaxSlowdown(parameters);
        LoadRecorder loadRecorder = extractLoadRecorder(parameters);
        return new LinearSlowdownStrategy(maxSlowdown, loadRecorder);
    }

    private static Duration extractMaxSlowdown(String[] parameters) {
        if (parameters.length >= 1) {
            return Duration.parse("PT" + parameters[0].toUpperCase());
        } else {
            return Duration.ofSeconds(60);
        }
    }

    private static LoadRecorder extractLoadRecorder(String[] parameters) {
        final int windowSize;
        if (parameters.length >= 2) {
            windowSize = (int) Duration.parse("PT" + parameters[1].toUpperCase()).getSeconds();
        } else {
            windowSize = 60;
        }
        return new CircularBufferRecorder(windowSize);
    }
}
