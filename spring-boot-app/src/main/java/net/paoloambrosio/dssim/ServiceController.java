package net.paoloambrosio.dssim;

import net.paoloambrosio.dssim.slowdown.SlowdownProvider;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
public class ServiceController {

    private final SlowdownProvider slowdownProvider;

    @Autowired
    public ServiceController(SlowdownProvider slowdownProvider) {
        this.slowdownProvider = slowdownProvider;
    }

    @RequestMapping("/")
    public ResponseEntity<String> root() throws InterruptedException {
        long requestSlowdownMs = slowdownProvider.computeSlowdown();
        Thread.sleep(requestSlowdownMs);
        return new ResponseEntity<>("s", HttpStatus.OK);
    }

    @ExceptionHandler(Throwable.class)
    public ResponseEntity<String> exceptionHandler() {
        return new ResponseEntity<>("x", HttpStatus.INTERNAL_SERVER_ERROR);
    }

}
