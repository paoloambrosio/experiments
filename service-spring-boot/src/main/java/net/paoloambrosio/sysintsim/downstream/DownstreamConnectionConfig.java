package net.paoloambrosio.sysintsim.downstream;

public interface DownstreamConnectionConfig {
    String getUrl();
    int getPoolSize();
    int getSocketTimeout();
    boolean isTcpNoDelay();
}
