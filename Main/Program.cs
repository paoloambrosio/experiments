using System.Reflection;
using ConfigSpike;
using Microsoft.Extensions.Configuration;

namespace Main;

static class Program
{
    public static void Main(string[] args)
    {
        var configBuilder = new ConfigurationBuilder();

        foreach (var assembly in LoadSolutionAssemblies())
        {
            var assemblyName = assembly.GetName().Name;
            var resourceName = $"{assemblyName}.Resources.appsettings.json";
            var stream = assembly.GetManifestResourceStream(resourceName);
            if (stream is not null)
            {
                Console.WriteLine($"Loaded {resourceName}");
                configBuilder.AddJsonStream(stream);
            }
        }

        var config = configBuilder
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
    
    private static Assembly[] LoadSolutionAssemblies()
    {
        return
            Directory.GetFiles(AppDomain.CurrentDomain.BaseDirectory, "*.dll")
            .Where(dll => !dll.Contains(@"\Microsoft.") && !dll.StartsWith(@"\System."))
            .Select(dll => Assembly.Load(AssemblyName.GetAssemblyName(dll)))
            .ToArray();
    }
}