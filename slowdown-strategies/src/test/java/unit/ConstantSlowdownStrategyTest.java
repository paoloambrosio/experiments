package unit;

import net.paoloambrosio.sysintsim.slowdown.ConstantSlowdownStrategy;
import org.junit.Test;

import java.time.Duration;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneOffset;

import static org.junit.Assert.*;

public class ConstantSlowdownStrategyTest {

    private static Instant T0 = Instant.ofEpochSecond(100);

    @Test
    public void returnsConstantTime() {
        Duration slowdown = Duration.ofMillis(1234);
        ConstantSlowdownStrategy constantSlowdownStrategy = new ConstantSlowdownStrategy(slowdown);
        assertEquals(slowdown, constantSlowdownStrategy.computeSlowdown(T0));
        assertEquals(slowdown, constantSlowdownStrategy.computeSlowdown(T0.plusSeconds(1)));
        assertEquals(slowdown, constantSlowdownStrategy.computeSlowdown(T0.plusSeconds(2)));
    }

    @Test
    public void buildsFromDurationString() {
        assertEquals(Duration.ofSeconds(2), ConstantSlowdownStrategy.build("2s").getSlowdown());
        assertEquals(Duration.ofMillis(123), ConstantSlowdownStrategy.build("0.123s").getSlowdown());
        assertEquals(Duration.ofMinutes(1), ConstantSlowdownStrategy.build("1m").getSlowdown());
        assertEquals(Duration.ofSeconds(1), ConstantSlowdownStrategy.build().getSlowdown());
    }

    @Test
    public void builderAcceptsAtMostOneParameter() {
        try {
            ConstantSlowdownStrategy.build("2s", "");
            fail("it should have thrown");
        } catch (IllegalArgumentException e) {
            assertEquals("At most one parameter expected (was: 2)", e.getMessage());
        }
    }
}