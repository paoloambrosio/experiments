package net.paoloambrosio.dssim.slowdown;

public class SlowdownStrategyFactory {

    // TODO Use reflection and look for classes implementing SlowdownStrategy with @Builder
    public static SlowdownStrategy fromDescription(String description) {
        String[] split = description.split(":", 2);
        String name = split[0];
        if (name.isEmpty()) {
            throw new IllegalArgumentException("Strategy name required");
        }
        final String[] parameters;
        if (split.length > 1) {
            parameters = split[1].split(":");
        } else {
            parameters = new String[0];
        }

        switch (name) {
            case "constant":
                return ConstantSlowdownStrategy.build(parameters);
            case "linear":
                return LinearSlowdownStrategy.build(parameters);
            default:
                throw new IllegalArgumentException("No strategy found for " + description);
        }
    }
}
