package unit;

import net.paoloambrosio.sysintsim.slowdown.CircularBufferRecorder;
import org.junit.Test;

import static org.hamcrest.CoreMatchers.is;
import static org.junit.Assert.fail;
import static utils.IsIntArrayContainingInOrder.intArrayContaining;
import static org.junit.Assert.assertThat;

public class CircularBufferRecorderTest {

    private final static long T0 = 100L;
    private final CircularBufferRecorder circularBufferRecorder = new CircularBufferRecorder(5);

    @Test
    public void mustHavePositiveSize() {
        try {
            new CircularBufferRecorder(0);
            fail("was constructed with zero size");
        } catch (IllegalArgumentException expected) {}
        try {
            new CircularBufferRecorder(-1);
            fail("was constructed with negative size");
        } catch (IllegalArgumentException expected) {}
    }

    @Test
    public void isConstructedAsStreamOfZeroes() {
        assertStreamIs(0, 0, 0, 0, 0);
    }

    @Test
    public void firstStoreAddsOneInLastPosition() {
        store(0);
        assertStreamIs(0, 0, 0, 0, 1);
        store(0, 0);
        assertStreamIs(0, 0, 0, 0, 3);
    }

    @Test
    public void secondStoreShiftsLastPosition() {
        store(0, 3, 3);
        assertStreamIs(0, 1, 0, 0, 2);
    }

    @Test
    public void storeOutOfWindowClearsLastPositionsSimple() {
        store(0, 1, 2, 3, 4, 5, 6, 7, 8, 9);
        assertStreamIs(1, 1, 1, 1, 1);
    }

    @Test
    public void storeOutOfWindowClearsLastPositions() {
        store(0, 3, 5, 5);
        assertStreamIs(0, 0, 1, 0, 2);
    }

    @Test
    public void storeMuchOutOfWindowClearsEverything() {
        store(0, 3, 12, 12);
        assertStreamIs(0, 0, 0, 0, 2);
    }

    private void store(int... ts) {
        for (int t : ts) {
            circularBufferRecorder.store(T0 + t);
        }
    }

    private void assertStreamIs(int... values) {
        assertThat(circularBufferRecorder.stream().toArray(), is(intArrayContaining(values)));
    }
}
