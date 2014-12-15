package net.paoloambrosio.wiremock;

import java.io.IOException;

import org.apache.http.client.fluent.Request;

public class ApacheHttpPriceService implements PriceService {

	public static final int SOCKET_TIMEOUT_MS = 2000;

	@Override
	public float fetchPrice() {
		try {
			String response = Request.Get("http://localhost:8080/price")
					.socketTimeout(SOCKET_TIMEOUT_MS)
					.execute().returnContent().asString();
			return Float.parseFloat(response);
		} catch (IOException e) {
			return Float.NaN;
		}
	}

}
