package net.paoloambrosio.dssim;

import net.paoloambrosio.dssim.slowdown.SlowdownProvider;
import net.paoloambrosio.dssim.slowdown.SlowdownProviderFactory;
import org.apache.catalina.connector.Connector;
import org.jmxtrans.embedded.EmbeddedJmxTrans;
import org.jmxtrans.embedded.config.ConfigurationParser;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.EnableAutoConfiguration;
import org.springframework.boot.context.embedded.EmbeddedServletContainerFactory;
import org.springframework.boot.context.embedded.tomcat.TomcatConnectorCustomizer;
import org.springframework.boot.context.embedded.tomcat.TomcatEmbeddedServletContainerFactory;
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

    @Value("${server.tomcat.nio}")
    private boolean useNioConnector;

    @Bean
    public EmbeddedServletContainerFactory servletContainer() {
        TomcatEmbeddedServletContainerFactory tomcat = new TomcatEmbeddedServletContainerFactory();
        if (!useNioConnector) {
            // Use BIO HTTP connector
            tomcat.setProtocol("org.apache.coyote.http11.Http11Protocol");
            // Disable HTTP KeepAlive
            tomcat.addConnectorCustomizers(new TomcatConnectorCustomizer() {
                @Override
                public void customize(Connector connector) {
                    connector.setAttribute("maxKeepAliveRequests", 1);
                }
            });
        }
        return tomcat;
    }
}