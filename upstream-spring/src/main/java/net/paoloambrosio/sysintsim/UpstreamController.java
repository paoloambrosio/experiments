package net.paoloambrosio.sysintsim;

import com.netflix.hystrix.exception.HystrixRuntimeException;
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

    private final DownstreamService downstreamService;

    @Autowired
    public UpstreamController(DownstreamService downstreamService) {
        this.downstreamService = downstreamService;
    }

    @RequestMapping("/")
    public ResponseEntity<String> index() {
        try {
            String downstreamResponse = downstreamService.call();
            HttpStatus status = "success".equals(downstreamResponse) ? HttpStatus.OK : HttpStatus.SERVICE_UNAVAILABLE;
            return new ResponseEntity<String>("success-" + downstreamResponse, status);
        } catch (SocketTimeoutException e) {
            return new ResponseEntity<String>("failure-timeout", HttpStatus.GATEWAY_TIMEOUT);
        } catch (IOException e) {
            return new ResponseEntity<String>("failure-unknown", HttpStatus.GATEWAY_TIMEOUT);
        } catch (HystrixRuntimeException e) {
            return new ResponseEntity<String>("failure-tripped", HttpStatus.GATEWAY_TIMEOUT);
        }
    }

}
