package net.paoloambrosio.sysintsim.controller;

import com.netflix.hystrix.exception.HystrixRuntimeException;
import net.paoloambrosio.sysintsim.service.DownstreamService;
import net.paoloambrosio.sysintsim.slowdown.SlowdownStrategy;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.io.IOException;
import java.net.SocketTimeoutException;
import java.time.Duration;
import java.time.Instant;

@RestController
public class UpstreamController {

    private final DownstreamService downstreamService;
    private final SlowdownStrategy slowdownStrategy;

    @Autowired
    public UpstreamController(DownstreamService downstreamService, SlowdownStrategy slowdownStrategy) {
        this.downstreamService = downstreamService;
        this.slowdownStrategy = slowdownStrategy;
    }

    // TODO Find a convention for response body and codes!
    @RequestMapping("/")
    public ResponseEntity<String> index() {
        try {
            Duration requestSlowdown = slowdownStrategy.computeSlowdown(Instant.now());
            String downstreamResponse = downstreamService.call();
            try {
                Thread.sleep(toMillis(requestSlowdown));
            } catch (InterruptedException e) {
                return new ResponseEntity<>("failure-interrupted", HttpStatus.SERVICE_UNAVAILABLE);
            }
            switch (downstreamResponse) {
                case "success":
                case "":
                    return new ResponseEntity<>("success", HttpStatus.OK);
                default:
                    return new ResponseEntity<>("failure", HttpStatus.SERVICE_UNAVAILABLE);
            }
        } catch (SocketTimeoutException e) {
            return new ResponseEntity<>("failure-timeout", HttpStatus.GATEWAY_TIMEOUT);
        } catch (IOException e) {
            return new ResponseEntity<>("failure-unknown", HttpStatus.GATEWAY_TIMEOUT);
        } catch (HystrixRuntimeException e) {
            return new ResponseEntity<>("failure-tripped", HttpStatus.GATEWAY_TIMEOUT);
        }
    }

    private long toMillis(Duration duration) {
        return duration.getSeconds() * 1_000 + duration.getNano() / 1_000_000;
    }

}
