package integration;

import static com.github.tomakehurst.wiremock.client.WireMock.aResponse;
import static com.github.tomakehurst.wiremock.client.WireMock.configureFor;
import static com.github.tomakehurst.wiremock.client.WireMock.get;
import static com.github.tomakehurst.wiremock.client.WireMock.givenThat;
import static com.github.tomakehurst.wiremock.client.WireMock.urlEqualTo;
import static com.tomakehurst.crashlab.saboteur.FirewallTimeout.firewallTimeout;
import static java.util.concurrent.TimeUnit.MILLISECONDS;
import static net.paoloambrosio.wiremock.ApacheHttpPriceService.CONNECTION_TIMEOUT_MS;
import static net.paoloambrosio.wiremock.ApacheHttpPriceService.SOCKET_TIMEOUT_MS;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import net.paoloambrosio.wiremock.ApacheHttpPriceService;

import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import com.github.tomakehurst.wiremock.client.WireMock;
import com.tomakehurst.crashlab.saboteur.Saboteur;

public class ApacheHttpPriceServiceTest {

	public static final String PRICE_SERVICE_HOST = "localhost";
	public static final int PRICE_SERVICE_PORT = 8080;

	public static final int TIMEOUT_TOLERANCE_MS = 500;

    private Saboteur priceServiceSaboteur = Saboteur.defineClient("test-price-service", PRICE_SERVICE_PORT, PRICE_SERVICE_HOST);

    private ApacheHttpPriceService priceService = new ApacheHttpPriceService();

	@BeforeClass
	public static void initWireMock() {
		configureFor(PRICE_SERVICE_HOST, PRICE_SERVICE_PORT);
	}

	@Before
	public void init() {
		priceServiceSaboteur.reset();
		WireMock.reset();
	}

	@Test
	public void returnsFixedPrice() {
		givenThat(get(urlEqualTo("/price"))
	            .willReturn(aResponse()
	            		.withStatus(200)
	            		.withBody("1.25")
	            		.withFixedDelay(SOCKET_TIMEOUT_MS - TIMEOUT_TOLERANCE_MS)));

	    assertEquals(1.25, priceService.fetchPrice(), 0.01);
	}

	@Test
	public void nanOnSlowResponse() {
		givenThat(get(urlEqualTo("/price"))
				.willReturn(aResponse()
						.withStatus(200)
		                .withBody("1.25")
		                .withFixedDelay(SOCKET_TIMEOUT_MS + TIMEOUT_TOLERANCE_MS)));

		float result = priceService.fetchPrice();
    	assertTrue("NaN expected, got " + result, Float.isNaN(result));
	}

	@Test
	public void nanOnConnectionTimeout() {
		givenThat(get(urlEqualTo("/price")).willReturn(
				aResponse().withStatus(200).withBody("1.25")));

		priceServiceSaboteur.addFault(
				firewallTimeout("price-service-firewall-timeout")
					.timeout(CONNECTION_TIMEOUT_MS + TIMEOUT_TOLERANCE_MS, MILLISECONDS));

		float result = priceService.fetchPrice();
		assertTrue("NaN expected, got " + result, Float.isNaN(result));
	}

}
