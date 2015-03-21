package net.paoloambrosio.sysintsim;

import org.apache.http.client.fluent.Executor;
import org.apache.http.client.fluent.Request;
import org.apache.http.config.SocketConfig;
import org.apache.http.impl.client.CloseableHttpClient;
import org.apache.http.impl.client.HttpClients;
import org.apache.http.impl.conn.PoolingHttpClientConnectionManager;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.io.IOException;

@RestController
public class UpstreamController {

    private final Executor executor;
    private final String downstreamUrl;

    @Autowired
    public UpstreamController(DownstreamConnectionConfig dcc) {
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

    @RequestMapping("/")
    public String index() {
        String downstreamResponse;
        try {
            downstreamResponse = executor.execute(Request.Get(downstreamUrl)).returnContent().asString();
        } catch (IOException e) {
            downstreamResponse = "unknown";
        }
        return "success-" + downstreamResponse;
    }

}
