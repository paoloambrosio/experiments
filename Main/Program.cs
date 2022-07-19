using AnotherDependency;
using ConfigSpike;
using Dependency;
using Microsoft.Extensions.Configuration;

namespace Main;

static class Program
{
    public static void Main(string[] args)
    {
        var configBuilder = new ConfigurationBuilder()
            .AddJsonFile("appsettings.json", optional: false, reloadOnChange: false);

        var plugins = new object[] {new DependencyPlugin(), new AnotherDependencyPlugin()};
        foreach (var assembly in plugins.Select(p => p.GetType().Assembly))
        {
            var assemblyName = assembly.GetName().Name;
            var resourceName = $"{assemblyName}.appsettings.json";
            var stream = assembly.GetManifestResourceStream(resourceName);
            if (stream is null) continue;
            Console.WriteLine($"Loaded {resourceName}");
            configBuilder.AddJsonStream(stream);
        }

        var config = configBuilder
            .AddJsonFile("usersettings.json", optional: true, reloadOnChange: true)
            .AddEnvironmentVariables()
            .AddCommandLine(args)
            .Build();

//        var settings = config.Bind<Settings>();
        var settings = config.Get<Settings>();
        Console.WriteLine("A: " + settings.A);
        Console.WriteLine("B: " + settings.B);
        Console.WriteLine("C: " + settings.C);
        Console.WriteLine("D: " + settings.D);
        // Console.WriteLine("Ip: " + settings.Ip);
        // Console.WriteLine("TimeSpan: " + settings.TimeSpan);
    }
}