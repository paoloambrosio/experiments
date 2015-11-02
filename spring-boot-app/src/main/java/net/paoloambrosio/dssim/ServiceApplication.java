package net.paoloambrosio.dssim;

import net.paoloambrosio.dssim.slowdown.SlowdownProvider;
import net.paoloambrosio.dssim.slowdown.SlowdownProviderFactory;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.EnableAutoConfiguration;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.ComponentScan;

@ComponentScan
@EnableAutoConfiguration
public class ServiceApplication {

    public static void main(String[] args) {
        SpringApplication.run(ServiceApplication.class, args);
    }

    @Value("${config.slowdown-strategy}")
    private String slowdownStrategy;

    @Bean
    public SlowdownProvider slowdownProvider() {
        return SlowdownProviderFactory.threadSafe(slowdownStrategy);
    }

}