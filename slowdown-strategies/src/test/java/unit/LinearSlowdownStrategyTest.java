package unit;

import net.paoloambrosio.sysintsim.slowdown.LinearSlowdownStrategy;
import net.paoloambrosio.sysintsim.slowdown.LoadRecorder;
import org.junit.Test;
import org.mockito.InOrder;

import java.time.Duration;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneOffset;
import java.util.stream.IntStream;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.when;

public class LinearSlowdownStrategyTest {

    private static long T0_EPOCH_SEC = 100L;
    private static Instant T0 = Instant.ofEpochSecond(T0_EPOCH_SEC);

    private static Duration MAX_SLOWDOWN = Duration.ofMillis(200);

    private LoadRecorder loadRecorderMock = mock(LoadRecorder.class);
    private LinearSlowdownStrategy linearSlowdownStrategy = new LinearSlowdownStrategy(MAX_SLOWDOWN, loadRecorderMock);

    @Test
    public void returnsAverageRequestsInMs() {
        when(loadRecorderMock.stream()).thenReturn(IntStream.of(0, 1, 2, 3, 4));

        assertEquals(Duration.ofMillis(2), linearSlowdownStrategy.computeSlowdown(T0));

        InOrder inOrder = inOrder(loadRecorderMock);
        inOrder.verify(loadRecorderMock).store(T0_EPOCH_SEC);
        inOrder.verify(loadRecorderMock).stream();
    }

    @Test
    public void builderAccaptsMaxSlowdownParameter() {
        assertEquals(Duration.ofSeconds(60), LinearSlowdownStrategy.build().getMaxSlowdown());
        assertEquals(Duration.ofSeconds(15), LinearSlowdownStrategy.build("15s").getMaxSlowdown());
        assertEquals(Duration.ofMinutes(2), LinearSlowdownStrategy.build("2m").getMaxSlowdown());
        assertEquals(Duration.ofMillis(200), LinearSlowdownStrategy.build("0.200s").getMaxSlowdown());
    }

    @Test
    public void builderAccaptsWindowSizeParameter() {
        assertEquals(10, LinearSlowdownStrategy.build("2m", "10s").getLoadRecorder().getWindowSize());
        assertEquals(15, LinearSlowdownStrategy.build("2m", "15.200s").getLoadRecorder().getWindowSize());
        assertEquals(60, LinearSlowdownStrategy.build("2m", "1m").getLoadRecorder().getWindowSize());
    }

    @Test
    public void builderAcceptsOneParameter() {
        try {
            LinearSlowdownStrategy.build("2s", "1m", "");
            fail("it should have thrown");
        } catch (IllegalArgumentException e) {
            assertEquals("At most two parameters expected (was: 3)", e.getMessage());
        }
    }

}