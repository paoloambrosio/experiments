package net.paoloambrosio.sysintsim.controller;

import com.netflix.hystrix.exception.HystrixRuntimeException;
import net.paoloambrosio.sysintsim.downstream.DownstreamService;
import net.paoloambrosio.sysintsim.slowdown.SlowdownProvider;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.io.IOException;
import java.net.SocketTimeoutException;

@RestController
public class ServiceController {

    private final DownstreamService downstreamService;
    private final SlowdownProvider slowdownProvider;

    @Autowired
    public ServiceController(DownstreamService downstreamService, SlowdownProvider slowdownProvider) {
        this.downstreamService = downstreamService;
        this.slowdownProvider = slowdownProvider;
    }

    // TODO Find a convention for response body and codes!
    @RequestMapping("/")
    public ResponseEntity<String> index() {
        try {
            long requestSlowdownMs = slowdownProvider.computeSlowdown();
            String downstreamResponse = downstreamService.call();
            try {
                Thread.sleep(requestSlowdownMs);
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
            return new ResponseEntity<>("failure-timeout", HttpStatus.SERVICE_UNAVAILABLE);
        } catch (IOException e) {
            return new ResponseEntity<>("failure-unknown: " + e.getMessage(), HttpStatus.SERVICE_UNAVAILABLE);
        } catch (HystrixRuntimeException e) {
            return new ResponseEntity<>("failure-tripped", HttpStatus.SERVICE_UNAVAILABLE);
        }
    }

}
