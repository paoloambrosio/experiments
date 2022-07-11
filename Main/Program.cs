using ConfigSpike;
using Microsoft.Extensions.Configuration;

namespace Main;

static class Program
{
    public static void Main(string[] args)
    {
        Console.WriteLine("Resources");
        foreach (var manifestResourceName in typeof(Program).Assembly.GetManifestResourceNames())
        {
            Console.WriteLine("- " + manifestResourceName);
        }

        var config = new ConfigurationBuilder()
            .AddJsonStream(typeof(Program).Assembly.GetManifestResourceStream("appsettings.json"))
            .AddJsonFile("appsettings.json", optional: false, reloadOnChange: true)
            .AddJsonFile("usersettings.json", optional: true)
            .AddEnvironmentVariables()
            .AddCommandLine(args)
            .Build();

        var section = config.GetSection(nameof(Settings));

        // var observable = ObservableConfig<Settings>.Create(section);
        // Console.WriteLine("Print the current value and wait for a change...");
        // observable.Take(2).ForEachAsync(settings => Console.WriteLine(settings.A)).Wait();

        var settings = section.Get<Settings>();
        Console.WriteLine("A: " + settings.A);
        Console.WriteLine("B: " + settings.B);
        Console.WriteLine("C: " + settings.C);
        Console.WriteLine("D: " + settings.D);
    }
}