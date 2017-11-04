package example;

import org.junit.jupiter.api.Test;
import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class LibraryTest {

    @Test public void testSomeLibraryMethod() {
        INDArray x = Nd4j.create(new float[]{1,1,1},new int[]{1, 3});
        INDArray y = Nd4j.create(new float[]{1,1,1,-1,-1,-1},new int[]{2, 3});
        Library library = new Library(y);

        assertEquals(Nd4j.create(new float[]{1,-1},new int[]{2, 1}), library.cosineSimilarity(x));
    }
}
