package example;

import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;
import org.openjdk.jmh.annotations.*;

import java.util.concurrent.TimeUnit;

@Fork(value = 2, warmups = 2)
public class LibraryBenchmark {

    @State(Scope.Thread)
    public static class MyState {
        INDArray a = Nd4j.rand(1, 14);
        INDArray b = Nd4j.rand(20000, 14);

        Library library = new Library(b);
    }

    @Benchmark
    @BenchmarkMode(Mode.AverageTime) @OutputTimeUnit(TimeUnit.MICROSECONDS)
    public void cosineSimilarity(MyState s) {
        s.library.cosineSimilarity(s.a);
    }
}
