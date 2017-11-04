package example;

import org.nd4j.linalg.api.ndarray.INDArray;

public class Library {
    public INDArray cosineSimilarity(INDArray x, INDArray y) {
        INDArray yt = y.transpose();
        INDArray xn = x.norm2(1);
        INDArray yn = y.norm2(1);

        return x.mmul(yt).div(xn.mmul(yn));
    }
}
