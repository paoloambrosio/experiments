package net.paoloambrosio.sysintsim.downstream;

import java.io.IOException;

public interface DownstreamService {

    public String call() throws IOException;
}
