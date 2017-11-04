package example;

import org.openjdk.jmh.annotations.*;

public class LibraryBenchmark {

    @State(Scope.Thread)
    public static class MyState {
        Library library = new Library();
    }

    @Benchmark
    @BenchmarkMode(Mode.AverageTime)
    public void testSomeMethod(MyState state) {
        state.library.someLibraryMethod();
    }
}
