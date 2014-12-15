package integration;

import static com.github.tomakehurst.wiremock.client.WireMock.aResponse;
import static com.github.tomakehurst.wiremock.client.WireMock.get;
import static com.github.tomakehurst.wiremock.client.WireMock.givenThat;
import static com.github.tomakehurst.wiremock.client.WireMock.urlEqualTo;
import static net.paoloambrosio.wiremock.ApacheHttpPriceService.SOCKET_TIMEOUT_MS;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import net.paoloambrosio.wiremock.ApacheHttpPriceService;

import org.junit.Rule;
import org.junit.Test;

import com.github.tomakehurst.wiremock.junit.WireMockRule;

public class ApacheHttpPriceServiceTest {

	public static final int PRICE_SERVICE_PORT = 8080;
	public static final int TIMEOUT_TOLERANCE_MS = 500;

    @Rule
    public WireMockRule wireMockRule = new WireMockRule(PRICE_SERVICE_PORT);

    private ApacheHttpPriceService priceService = new ApacheHttpPriceService();

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

}
