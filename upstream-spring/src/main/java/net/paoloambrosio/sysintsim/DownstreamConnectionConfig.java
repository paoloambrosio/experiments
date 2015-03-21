package net.paoloambrosio.sysintsim;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.stereotype.Component;

@Component
//@ConfigurationProperties(prefix="service.downstream")
public class DownstreamConnectionConfig {

    @Value("${service.downstream.url}")
    private String url;

    @Value("${service.downstream.pool-size}")
    private int poolSize;

    @Value("${service.downstream.socket-timeout}")
    private int socketTimeout;

    //@Value("${service.downstream.tcp-no-delay}")
    private boolean tcpNoDelay; // Disable Nagle

    public String getUrl() {
        return url;
    }

    public int getPoolSize() {
        return poolSize;
    }

    public int getSocketTimeout() {
        return socketTimeout;
    }

    public boolean isTcpNoDelay() {
        return tcpNoDelay;
    }
}
