package net.paoloambrosio.sysintsim.slowdown;


import java.time.Duration;
import java.time.Instant;

public final class ConstantSlowdownStrategy implements SlowdownStrategy {

    private final Duration slowdown;

    public ConstantSlowdownStrategy(Duration slowdown) {
        this.slowdown = slowdown;
    }

    /**
     * @return configured slowdown parameter
     */
    public Duration getSlowdown() {
        return slowdown;
    }

    @Override
    public Duration computeSlowdown(Instant t) {
        return slowdown;
    }

    /**
     * Builds a {@link net.paoloambrosio.sysintsim.slowdown.ConstantSlowdownStrategy} from a duration string.
     *
     * @param parameters slowdown in seconds (optional, default 1s)
     * @return strategy
     */
    @Builder("constant")
    public static ConstantSlowdownStrategy build(String... parameters) {
        if (parameters.length > 1) {
            String message = String.format("At most one parameter expected (was: %d)", parameters.length);
            throw new IllegalArgumentException(message);
        }
        final Duration constantDuration = extractSlowdown(parameters);
        return new ConstantSlowdownStrategy(constantDuration);
    }

    private static Duration extractSlowdown(String[] parameters) {
        if (parameters.length >= 1) {
            return Duration.parse("PT" + parameters[0].toUpperCase());
        } else {
            return Duration.ofSeconds(1);
        }
    }
}
