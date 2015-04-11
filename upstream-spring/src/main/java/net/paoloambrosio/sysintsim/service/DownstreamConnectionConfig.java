package net.paoloambrosio.sysintsim.service;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.stereotype.Component;

public interface DownstreamConnectionConfig {
    String getUrl();
    int getPoolSize();
    int getSocketTimeout();
    boolean isTcpNoDelay();
}
