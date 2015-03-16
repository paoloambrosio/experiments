package net.paoloambrosio.sysintsim;

import org.apache.http.client.fluent.Request;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.io.IOException;

@RestController
public class UpstreamController {

    @RequestMapping("/")
    public String index() {
        String downstreamResponse;
        try {
            downstreamResponse = Request.Get("http://localhost:9000/")
                    .connectTimeout(1000)
                    .socketTimeout(5000)
                    .execute().returnContent().asString();
        } catch (IOException e) {
            downstreamResponse = "unknown";
        }
        return "success-" + downstreamResponse;
    }

}
