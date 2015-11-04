package net.paoloambrosio.dssim;

import net.paoloambrosio.dssim.slowdown.SlowdownProvider;
import net.paoloambrosio.dssim.slowdown.SlowdownProviderFactory;
import org.jmxtrans.embedded.EmbeddedJmxTrans;
import org.jmxtrans.embedded.config.ConfigurationParser;
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

    @Bean
    public EmbeddedJmxTrans jmxTrans() throws Exception {
        // FIXME Don't like this!
        System.setProperty("graphite.host", System.getenv("GRAPHITE_PORT_2003_TCP_ADDR"));
        System.setProperty("graphite.port", System.getenv("GRAPHITE_PORT_2003_TCP_PORT"));

        EmbeddedJmxTrans bean = new ConfigurationParser().newEmbeddedJmxTrans("classpath:jmxtrans.json");
        bean.start();
        return bean;
    }
}