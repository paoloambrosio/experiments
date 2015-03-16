package net.paoloambrosio.sysintsim;

import org.jmxtrans.embedded.config.ConfigurationParser;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.EnableAutoConfiguration;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.Configuration;

@Configuration
@EnableAutoConfiguration
@ComponentScan
public class UpstreamApplication {

  public static void main(String[] args) throws Exception {
    new ConfigurationParser().newEmbeddedJmxTrans("classpath:jmxtrans.json").start();
    SpringApplication.run(UpstreamApplication.class, args);
  }
}
