package net.paoloambrosio;

import org.apache.kafka.common.serialization.Serdes;
import org.apache.kafka.streams.Consumed;
import org.apache.kafka.streams.KafkaStreams;
import org.apache.kafka.streams.StreamsBuilder;
import org.apache.kafka.streams.StreamsConfig;
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

        KafkaStreams streams = new KafkaStreams(builder.build(), config);
        streams.start();
    }

}