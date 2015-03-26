package net.paoloambrosio.sysintsim;

import org.apache.http.client.fluent.Executor;
import org.apache.http.client.fluent.Request;
import org.apache.http.config.SocketConfig;
import org.apache.http.impl.client.CloseableHttpClient;
import org.apache.http.impl.client.HttpClients;
import org.apache.http.impl.conn.PoolingHttpClientConnectionManager;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.io.IOException;
import java.net.SocketTimeoutException;

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
    public ResponseEntity<String> index() {
        try {
            String downstreamResponse = executor.execute(Request.Get(downstreamUrl)).returnContent().asString();
            HttpStatus status = "success".equals(downstreamResponse) ? HttpStatus.OK : HttpStatus.SERVICE_UNAVAILABLE;
            return new ResponseEntity<String>("success-" + downstreamResponse, status);
        } catch (SocketTimeoutException e) {
            return new ResponseEntity<String>("success-timeout", HttpStatus.GATEWAY_TIMEOUT);
        } catch (IOException e) {
            return new ResponseEntity<String>("success-unknown", HttpStatus.GATEWAY_TIMEOUT);
        }
    }

}
