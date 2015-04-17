package net.paoloambrosio.sysintsim.spring;

import com.netflix.hystrix.contrib.javanica.aop.aspectj.HystrixCommandAspect;
import net.paoloambrosio.sysintsim.downstream.ApacheHttpClientDownstreamService;
import net.paoloambrosio.sysintsim.downstream.DownstreamConnectionConfig;
import net.paoloambrosio.sysintsim.downstream.DownstreamService;
import net.paoloambrosio.sysintsim.slowdown.SlowdownProvider;
import net.paoloambrosio.sysintsim.slowdown.SlowdownProviderFactory;
import org.apache.commons.lang.StringUtils;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;

public class DownstreamServiceConfig {

    @Value("${service.downstream.url}")
    private String url;

    @Value("${service.downstream.pool-size}")
    private int poolSize;

    @Value("${service.downstream.socket-timeout}")
    private int socketTimeout;

    //@Value("${service.downstream.tcp-no-delay}")
    private boolean tcpNoDelay; // Disable Nagle

    private DownstreamConnectionConfig downstreamConnectionConfig() {
        return new DownstreamConnectionConfig() {
            @Override public String getUrl() { return url; }
            @Override public int getPoolSize() { return poolSize; }
            @Override public int getSocketTimeout() { return socketTimeout; }
            @Override public boolean isTcpNoDelay() { return tcpNoDelay; }
        };
    }

    @Bean
    public DownstreamService downstreamService() {
        if (StringUtils.isEmpty(url)) {
            return () -> "";
        } else {
            return new ApacheHttpClientDownstreamService(downstreamConnectionConfig());
        }
    }

    @Value("${config.slowdown-strategy}")
    private String slowdownStrategy;

    @Bean
    public SlowdownProvider slowdownProvider() {
        return SlowdownProviderFactory.threadSafe(slowdownStrategy);
    }


    @Value("${config.enable.circuit-breaker}")
    public boolean circuitBreakerEnabled;

    @Bean
    public Object hystrixAspect() {
        if (circuitBreakerEnabled) {
            return new HystrixCommandAspect();
        } else {
            return null;
        }
    }

}
