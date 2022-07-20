using System.ComponentModel;
using System.ComponentModel.Design.Serialization;
using System.Globalization;
using System.Net;

namespace Main;

public class IpAddressConverter : TypeConverter
  {
    public override bool CanConvertFrom(ITypeDescriptorContext? context, Type sourceType) => sourceType == typeof (string) || base.CanConvertFrom(context, sourceType);

    public override bool CanConvertTo(ITypeDescriptorContext? context, Type? destinationType) => destinationType == typeof (InstanceDescriptor) || base.CanConvertTo(context, destinationType);

    public override object? ConvertFrom(
      ITypeDescriptorContext? context,
      CultureInfo? culture,
      object value)
    {
      if (!(value is string))
        return base.ConvertFrom(context, culture, value);
      var input = ((string) value).Trim();
      try
      {
        return IPAddress.Parse(input);
      }
      catch (FormatException ex)
      {
        throw new FormatException("Foo", ex);
      }
    }
  }