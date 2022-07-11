using System.Dynamic;
using System.Reactive;
using System.Reactive.Subjects;
using Microsoft.Extensions.Configuration;

namespace ConfigSpike;

public class ObservableConfig<T> : ObservableBase<T>
{
    private IConfigurationSection _configurationSection;

    private ObservableConfig(IConfigurationSection configurationSection)
    {
        _configurationSection = configurationSection;
    }

    protected override IDisposable SubscribeCore(IObserver<T> observer)
    {
        return _configurationSection.GetReloadToken().RegisterChangeCallback(o =>
        {
            var t = _configurationSection.Get<T>();
            observer.OnNext(t);
        }, null);
    }

    public static IObservable<T> Create(IConfigurationSection configurationSection)
    {
        var initialValue = configurationSection.Get<T>();
        var oc = new ObservableConfig<T>(configurationSection);
        var bs = new BehaviorSubject<T>(initialValue);
        oc.Subscribe(bs);
        return bs;
    }
}