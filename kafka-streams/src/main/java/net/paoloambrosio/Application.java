package net.paoloambrosio;

import io.opentracing.Tracer;
import io.opentracing.contrib.kafka.streams.TracingKafkaClientSupplier;
import org.apache.kafka.common.serialization.Serdes;
import org.apache.kafka.streams.*;
import org.apache.kafka.streams.kstream.Produced;

import java.util.Properties;

public class Application {

    public static void main(final String[] args) {
        Properties config = new Properties();
        config.put(StreamsConfig.APPLICATION_ID_CONFIG, "wordcount-application");
        config.put(StreamsConfig.BOOTSTRAP_SERVERS_CONFIG, "kafka:9092");

        StreamsBuilder builder = new StreamsBuilder();
        builder.stream("lines", Consumed.with(Serdes.String(), Serdes.String()))
                .mapValues(textLine -> String.valueOf(textLine.length()))
                .to("lineslength", Produced.with(Serdes.String(), Serdes.String()));

        Tracer tracer = com.uber.jaeger.Configuration.fromEnv().getTracer();
        KafkaClientSupplier supplier = new TracingKafkaClientSupplier(tracer);
        KafkaStreams streams = new KafkaStreams(builder.build(), new StreamsConfig(config), supplier);
        streams.start();
    }

}