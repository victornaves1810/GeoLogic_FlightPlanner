def classFactory(iface):
    from .geologic_provider import GeoLogicProviderPlugin
    return GeoLogicProviderPlugin()