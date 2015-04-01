package net.paoloambrosio.sysintsim;

import com.netflix.hystrix.contrib.javanica.aop.aspectj.HystrixCommandAspect;
import org.jmxtrans.embedded.EmbeddedJmxTrans;
import org.jmxtrans.embedded.config.ConfigurationParser;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.EnableAutoConfiguration;
import org.springframework.context.annotation.*;
import org.springframework.core.type.AnnotatedTypeMetadata;

@Configuration
@EnableAutoConfiguration
@ComponentScan
public class UpstreamApplication {

    public static class CircuitBreakerEnabledCondition implements Condition {
        @Override
        public boolean matches(ConditionContext context, AnnotatedTypeMetadata metadata) {
            return context.getEnvironment().getProperty("config.enable.circuit-breaker", Boolean.TYPE);
        }
    }

    @Bean
    @Conditional(CircuitBreakerEnabledCondition.class)
    public HystrixCommandAspect hystrixAspect() {
        return new HystrixCommandAspect();
    }

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
