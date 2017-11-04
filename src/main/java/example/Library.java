package example;

import org.nd4j.linalg.api.ndarray.INDArray;

public class Library {

    final INDArray yt;
    final INDArray yn;

    public Library(INDArray y) {
        yt = y.transpose();
        yn = y.norm2(1);
    }

    public INDArray cosineSimilarity(INDArray x) {
        INDArray xn = x.norm2(1);
        return x.mmul(yt).div(xn.mmul(yn));
    }
}
