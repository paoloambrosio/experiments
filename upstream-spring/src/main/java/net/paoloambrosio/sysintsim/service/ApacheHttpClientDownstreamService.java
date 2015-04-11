package net.paoloambrosio.sysintsim.service;

import com.netflix.hystrix.contrib.javanica.annotation.HystrixCommand;
import org.apache.http.client.fluent.Executor;
import org.apache.http.client.fluent.Request;
import org.apache.http.config.SocketConfig;
import org.apache.http.impl.client.CloseableHttpClient;
import org.apache.http.impl.client.HttpClients;
import org.apache.http.impl.conn.PoolingHttpClientConnectionManager;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;

import java.io.IOException;

public class ApacheHttpClientDownstreamService implements DownstreamService {

    private final Executor executor;
    private final String downstreamUrl;

    @Autowired
    public ApacheHttpClientDownstreamService(DownstreamConnectionConfig dcc) {
        PoolingHttpClientConnectionManager cm = new PoolingHttpClientConnectionManager();
        cm.setMaxTotal(dcc.getPoolSize());
        cm.setDefaultMaxPerRoute(dcc.getPoolSize());
        cm.setDefaultSocketConfig(SocketConfig.custom()
                        .setSoKeepAlive(true)
                        .setSoTimeout(dcc.getSocketTimeout())
                        .setTcpNoDelay(dcc.isTcpNoDelay())
                        .build()
        );
        CloseableHttpClient client = HttpClients.custom().setConnectionManager(cm).build();
        executor = Executor.newInstance(client);
        downstreamUrl = dcc.getUrl();
    }

    @HystrixCommand
    public String call() throws IOException {
        return executor.execute(Request.Get(downstreamUrl)).returnContent().asString();
    }

}
