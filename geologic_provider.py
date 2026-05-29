import os
from qgis.core import QgsProcessingProvider
from qgis.PyQt.QtGui import QIcon
from .geologic_algorithm import (
    DjiAdvancedWaypointAlgorithm,
    DjiSimpleWaypointAlgorithm
)


class GeoLogicProvider(QgsProcessingProvider):
    def loadAlgorithms(self, *args, **kwargs):
        # Agora o plugin carrega as DUAS ferramentas
        self.addAlgorithm(DjiAdvancedWaypointAlgorithm())
        self.addAlgorithm(DjiSimpleWaypointAlgorithm())

    def id(self, *args, **kwargs):
        return 'geologic_tools'

    def name(self, *args, **kwargs):
        return 'GeoLogic Flight Tools'

    def icon(self):
        # Chama o ícone icon.png que está na mesma pasta do plugin
        path = os.path.join(os.path.dirname(__file__), 'icon.png')
        return QIcon(path)


class GeoLogicProviderPlugin(object):
    def __init__(self):
        self.provider = GeoLogicProvider()

    def initProcessing(self):
        from qgis.core import QgsApplication
        QgsApplication.processingRegistry().addProvider(self.provider)

    def initGui(self):
        self.initProcessing()

    def unload(self):
        from qgis.core import QgsApplication
        QgsApplication.processingRegistry().removeProvider(self.provider)
