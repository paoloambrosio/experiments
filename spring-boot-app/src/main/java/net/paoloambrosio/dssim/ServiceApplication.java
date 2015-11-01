package net.paoloambrosio.dssim;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.EnableAutoConfiguration;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@EnableAutoConfiguration
public class ServiceApplication {

    @RequestMapping("/")
    public ResponseEntity<String> root() {
        return new ResponseEntity<>("s", HttpStatus.OK);
    }

    @ExceptionHandler(Throwable.class)
    public ResponseEntity<String> exceptionHandler() {
        return new ResponseEntity<>("x", HttpStatus.INTERNAL_SERVER_ERROR);
    }

    public static void main(String[] args) {
        SpringApplication.run(ServiceApplication.class, args);
    }
}