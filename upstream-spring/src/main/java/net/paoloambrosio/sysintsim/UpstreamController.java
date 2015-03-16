package net.paoloambrosio.sysintsim;

import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.bind.annotation.RequestMapping;

@RestController
public class UpstreamController {

    @RequestMapping("/")
    public String index() {
        return "upstream-success";
    }

}
