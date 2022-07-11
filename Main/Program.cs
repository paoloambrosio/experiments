using System.Reactive.Linq;
using ConfigSpike;
using Microsoft.Extensions.Configuration;

// Build a config object, using env vars and JSON providers.
var config = new ConfigurationBuilder()
    .AddJsonFile("appsettings.json", optional: false, reloadOnChange: true)
    .AddJsonFile("usersettings.json", optional: true)
    .AddEnvironmentVariables()
    .AddCommandLine(args)
    .Build();

// var gameConfig = new ConfigurationBuilder()
//     .AddConfiguration(config)
//     .AddJsonFile("gamesettings.json")
//     .Build();

// Get values from the config given their key and their target type.
//var settings = config.GetValue<Settings>(nameof(Settings));
var section = config.GetSection(nameof(Settings));
// var settings = section.Get<Settings>();
// var boundSettings = new Settings();

// Console.WriteLine(settings.KeyOne);      // 1
// Console.WriteLine(boundSettings.KeyOne); // 0

// Console.WriteLine($"KeyOne = {settings.KeyOne}");
// Console.WriteLine($"KeyTwo = {settings.KeyTwo}");
// Console.WriteLine($"KeyThree:Message = {settings.KeyThree?.Message}");

// var promise = new TaskCompletionSource<Settings>();
// section.Bind(boundSettings);
// var reloadTokenDisposable = section.GetReloadToken().RegisterChangeCallback(o =>
// {
//     Console.WriteLine("Done!");
//     promise.SetResult(section.Get<Settings>());
// }, null);

var observable = ObservableConfig<Settings>.Create(section);

// promise.Task.Wait();

// // config and section are automatically reloaded, but the bindings are not
// Console.WriteLine(settings.KeyOne);            // 1
// Console.WriteLine(boundSettings.KeyOne);       // 1
// Console.WriteLine(promise.Task.Result.KeyOne); // 2 (or whatever it changed to)

// reloadTokenDisposable.Dispose();

Console.WriteLine("Print the current value and wait for a change...");
observable.Take(2).ForEachAsync(settings => Console.WriteLine(settings.KeyOne)).Wait();
