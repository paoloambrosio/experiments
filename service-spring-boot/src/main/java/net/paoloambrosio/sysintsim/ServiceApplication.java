package net.paoloambrosio.sysintsim;

import net.paoloambrosio.sysintsim.spring.DownstreamServiceConfig;
import org.jmxtrans.embedded.EmbeddedJmxTrans;
import org.jmxtrans.embedded.config.ConfigurationParser;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.EnableAutoConfiguration;
import org.springframework.context.annotation.*;

@Configuration
@EnableAutoConfiguration
@ComponentScan
@Import(DownstreamServiceConfig.class)
public class ServiceApplication {

    @Bean
    public EmbeddedJmxTrans jmxTrans() throws Exception {
        EmbeddedJmxTrans bean = new ConfigurationParser().newEmbeddedJmxTrans("classpath:jmxtrans.json");
        bean.start();
        return bean;
    }

    public static void main(String[] args) {
        SpringApplication.run(ServiceApplication.class, args);
    }
}
