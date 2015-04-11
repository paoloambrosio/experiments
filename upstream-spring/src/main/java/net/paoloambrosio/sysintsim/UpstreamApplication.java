package net.paoloambrosio.sysintsim;

import com.netflix.hystrix.contrib.javanica.aop.aspectj.HystrixCommandAspect;
import net.paoloambrosio.sysintsim.service.ServiceSpringConfig;
import org.jmxtrans.embedded.EmbeddedJmxTrans;
import org.jmxtrans.embedded.config.ConfigurationParser;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.EnableAutoConfiguration;
import org.springframework.context.annotation.*;
import org.springframework.context.support.PropertySourcesPlaceholderConfigurer;
import org.springframework.core.type.AnnotatedTypeMetadata;

@Configuration
@EnableAutoConfiguration
@ComponentScan
@Import(ServiceSpringConfig.class)
public class UpstreamApplication {

    @Bean
    public EmbeddedJmxTrans jmxTrans() throws Exception {
        EmbeddedJmxTrans bean = new ConfigurationParser().newEmbeddedJmxTrans("classpath:jmxtrans.json");
        bean.start();
        return bean;
    }

    public static void main(String[] args) {
        SpringApplication.run(UpstreamApplication.class, args);
    }
}
