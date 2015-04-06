package net.paoloambrosio.sysintsim.slowdown;

import javax.annotation.concurrent.NotThreadSafe;
import java.time.Duration;
import java.util.Arrays;
import java.util.stream.IntStream;

@NotThreadSafe
public class CircularBufferRecorder implements LoadRecorder {

    private final int[] buffer;
    private final int windowSize;
    private int bufferStart;
    private int bufferEnd;
    private long latestTime;

    public CircularBufferRecorder(int windowSize) {
        if (windowSize <= 0) {
            throw new IllegalArgumentException("Window size must be positive!");
        }
        this.windowSize = windowSize;
        this.buffer = new int[windowSize];
        bufferStart = 0;
        bufferEnd = windowSize - 1;
        latestTime = 0;
    }

    @Override
    public int getWindowSize() {
        return windowSize;
    }

    @Override
    public CircularBufferRecorder store(long t) {
        if (t > latestTime) {
            shiftTo(t);
        }
        buffer[bufferEnd]++;
        return this;
    }

    private void shiftTo(long newTime) {
        final int shift = (int) (newTime - latestTime);

        if (bufferStart + shift < windowSize) {
            Arrays.fill(buffer, bufferStart, bufferStart + shift, 0);
        } else if (shift >= windowSize) {
            Arrays.fill(buffer, 0, windowSize, 0);
        } else {
            Arrays.fill(buffer, bufferStart, windowSize, 0);
            Arrays.fill(buffer, 0, bufferStart + shift - windowSize, 0);
        }

        bufferStart = (bufferStart + shift) % windowSize;
        bufferEnd = (bufferEnd + shift) % windowSize;

        latestTime = newTime;
    }

    @Override
    public IntStream stream() {
        return IntStream.concat(Arrays.stream(buffer, bufferStart, windowSize), Arrays.stream(buffer, 0, bufferStart));
    }
}
