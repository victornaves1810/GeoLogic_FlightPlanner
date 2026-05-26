from qgis.PyQt.QtCore import QCoreApplication, QVariant
from qgis.PyQt.QtGui import QFont, QColor
from qgis.core import (Qgis, QgsProcessing, QgsProcessingAlgorithm, 
                       QgsProcessingParameterFeatureSource,
                       QgsProcessingParameterRasterLayer,
                       QgsProcessingParameterNumber,
                       QgsProcessingParameterBoolean,
                       QgsProcessingParameterFileDestination,
                       QgsProcessingParameterFeatureSink,
                       QgsFeature, QgsGeometry, QgsPointXY,
                       QgsWkbTypes, QgsFeatureSink, QgsFields, QgsField,
                       QgsCoordinateTransform, QgsCoordinateReferenceSystem, QgsUnitTypes,
                       QgsDistanceArea, QgsProcessingLayerPostProcessorInterface,
                       QgsLineSymbol, QgsMarkerSymbol, QgsSingleSymbolRenderer,
                       QgsCategorizedSymbolRenderer, QgsRendererCategory,
                       QgsPalLayerSettings, QgsVectorLayerSimpleLabeling, 
                       QgsTextFormat, QgsTextBufferSettings)
import os
import tempfile
import zipfile
import math
import traceback
import time

# ==========================================
# VISUAL POST-PROCESSOR CLASS
# ==========================================
class GeoLogicStylePostProcessor(QgsProcessingLayerPostProcessorInterface):
    def __init__(self, layer_type):
        super().__init__()
        self.layer_type = layer_type 

    def postProcessLayer(self, layer, context, feedback):
        if self.layer_type == 'lines':
            symbol = QgsLineSymbol.createSimple({
                'color': '0,170,255,255', 
                'width': '1.0', 
                'line_style': 'solid'
            })
            layer.setRenderer(QgsSingleSymbolRenderer(symbol))
            layer.setName("GeoLogic - Flight Route")
            
        elif self.layer_type == 'points':
            sym_start = QgsMarkerSymbol.createSimple({'name': 'triangle', 'color': '0,255,0,255', 'outline_color': '0,0,0,255', 'size': '3.2'})
            sym_end = QgsMarkerSymbol.createSimple({'name': 'square', 'color': '255,0,0,255', 'outline_color': '0,0,0,255', 'size': '3.2'})
            sym_normal = QgsMarkerSymbol.createSimple({'name': 'circle', 'color': '100,255,100,255', 'outline_color': '0,100,0,255', 'size': '2.5'})
            
            cat_start = QgsRendererCategory("Start", sym_start, "Start Point")
            cat_end = QgsRendererCategory("End", sym_end, "End Point")
            cat_normal = QgsRendererCategory("", sym_normal, "Waypoint")
            
            renderer = QgsCategorizedSymbolRenderer("label", [cat_start, cat_end, cat_normal])
            layer.setRenderer(renderer)
            layer.setName("GeoLogic - Waypoints")

            pal_set = QgsPalLayerSettings()
            pal_set.fieldName = "label"
            pal_set.isExpression = False
            pal_set.placement = Qgis.LabelPlacement.OverPoint
            pal_set.quadOffset = Qgis.LabelQuadrantPosition.QuadrantAbove
            pal_set.yOffset = -2
            
            text_format = QgsTextFormat()
            universal_font = QFont("sans-serif", 10)
            universal_font.setBold(True)
            text_format.setFont(universal_font)
            text_format.setColor(QColor("black"))
            
            text_buffer = QgsTextBufferSettings()
            text_buffer.setEnabled(True)
            text_buffer.setSize(1)
            text_buffer.setColor(QColor("white"))
            text_format.setBuffer(text_buffer)
            
            pal_set.setFormat(text_format)
            
            labeling = QgsVectorLayerSimpleLabeling(pal_set)
            layer.setLabelsEnabled(True)
            layer.setLabeling(labeling)
        
        layer.triggerRepaint()

# ==========================================
# SHARED FLIGHT LOGIC ENGINE
# ==========================================
def get_optimal_angle(geometry):
    try:
        min_height = float('inf')
        best_angle_deg = 0
        center = geometry.centroid().asPoint()
        for angle_deg in range(180):
            geom_clone = QgsGeometry(geometry)
            geom_clone.rotate(-angle_deg, center)
            bbox = geom_clone.boundingBox()
            height = bbox.yMaximum() - bbox.yMinimum()
            if height < min_height:
                min_height = height
                best_angle_deg = angle_deg
        return math.radians(best_angle_deg)
    except Exception: 
        return 0 

def process_flight_logic(algo_instance, context, feedback, source, dem_layer, flight_altitude, flight_speed, 
                         spacing_val, buffer_val, dens_val, use_custom_angle, azimuth_val, kmz_path, 
                         dest_id_lines, sink_lines, dest_id_points, sink_points, line_fields, point_fields, is_simple_mode=False):
    
    results = {'OUTPUT_LINES': dest_id_lines, 'OUTPUT_POINTS': dest_id_points}
    if not source: return results

    try:
        features = list(source.getFeatures())
        if not features: return results
        poly_geom = features[0].geometry()
        if poly_geom.isNull() or poly_geom.isEmpty(): return results
        
        is_geographic = source.sourceCrs().mapUnits() == QgsUnitTypes.DistanceDegrees
        conversion_factor = 111320.0 if is_geographic else 1.0
        
        spacing_step = spacing_val / conversion_factor
        
        if buffer_val > 0:
            buffer_step = buffer_val / conversion_factor
            poly_geom = poly_geom.buffer(buffer_step, 8)

        center = poly_geom.centroid().asPoint()
        
        if use_custom_angle:
            angle_deg = azimuth_val - 90.0
            angle_rad = math.radians(angle_deg)
            feedback.pushInfo(f"ℹ️ Orientation: Custom azimuth of {azimuth_val}° (Map rotation: {angle_deg}°).")
        else:
            angle_rad = get_optimal_angle(poly_geom)
            angle_deg = math.degrees(angle_rad)
            feedback.pushInfo(f"ℹ️ Orientation: Auto-calculated map angle of {angle_deg:.2f}° for efficiency.")

        poly_geom.rotate(-angle_deg, center)
        box = poly_geom.boundingBox()

        clipped_lines = []
        y = box.yMaximum()
        while y > box.yMinimum():
            grid_line = QgsGeometry.fromPolylineXY([QgsPointXY(box.xMinimum() - (1.0/conversion_factor), y), QgsPointXY(box.xMaximum() + (1.0/conversion_factor), y)])
            intersection = grid_line.intersection(poly_geom)
            if not intersection.isEmpty():
                if intersection.wkbType() == QgsWkbTypes.LineString:
                    clipped_lines.append(intersection)
                elif intersection.wkbType() == QgsWkbTypes.MultiLineString:
                    for l in intersection.asGeometryCollection():
                        clipped_lines.append(l)
            y -= spacing_step

        # ==========================================
        # RC2 MEMORY PROTECTION & SMART DENSIFICATION
        # ==========================================
        MAX_RC2_WP = 200       # Hardware limit for DJI RC2
        MAX_FLIGHT_TIME = 35.0 # Minutes safe limit
        
        if dem_layer is None:
            actual_dens_step = float('inf')
            feedback.pushInfo("ℹ️ RC2 Optimization: Flat flight (No DEM) detected. Intermediate waypoints disabled to save memory.")
        else:
            # Internal Simulator: Find the best dens_val to stay under 200 waypoints
            # Try steps from 10m up to 40m max
            test_dens = 10.0 if is_simple_mode else max(1.0, dens_val)
            final_wp_count = 0
            
            while test_dens <= 40.0:
                final_wp_count = 0
                for line_geom in clipped_lines:
                    pts = line_geom.asPolyline()
                    if len(pts) >= 2:
                        dist_m = math.hypot(pts[-1].x() - pts[0].x(), pts[-1].y() - pts[0].y()) * conversion_factor
                        # Points per line = (Distance / Densification) + 2 edges
                        final_wp_count += int(dist_m / test_dens) + 2
                
                if final_wp_count <= MAX_RC2_WP:
                    break 
                
                if test_dens >= 40.0:
                    break # Hard cap at 40m, even if we exceed 200 points
                
                test_dens += 1.0 # Increase spacing and test again
                
            actual_dens_step = test_dens / conversion_factor
            
            if final_wp_count > MAX_RC2_WP:
                feedback.pushWarning(f"⚠️ RC2 Alert: Route is massive. Densification capped at 40.0m, but total waypoints ({final_wp_count}) exceeds safe limit of {MAX_RC2_WP}. App may be unstable.")
            elif test_dens > (10.0 if is_simple_mode else dens_val):
                feedback.pushWarning(f"⚠️ RC2 Protection: Densification spacing dynamically increased to {test_dens:.1f}m to keep waypoints under {MAX_RC2_WP} (Total: {final_wp_count} pts).")
            else:
                feedback.pushInfo(f"ℹ️ RC2 Safe: Using {test_dens:.1f}m densification (Estimated: {final_wp_count} total waypoints).")

        waypoints = []
        go_right = True
        for line_geom in clipped_lines:
            line_geom.rotate(angle_deg, center)
            points = line_geom.asPolyline()
            if not go_right: points.reverse()
            if len(points) >= 2:
                p_start, p_end = points[0], points[-1]
                total_dist = math.hypot(p_end.x() - p_start.x(), p_end.y() - p_start.y())
                
                if actual_dens_step == float('inf'):
                    num_segments = 1
                else:
                    num_segments = int(total_dist / actual_dens_step) + 1
                    
                for i in range(num_segments):
                    frac = i / num_segments
                    x_inter = p_start.x() + (p_end.x() - p_start.x()) * frac
                    y_inter = p_start.y() + (p_end.y() - p_start.y()) * frac
                    waypoints.append(QgsPointXY(x_inter, y_inter))
                waypoints.append(p_end)
            go_right = not go_right

        if not waypoints: return results

        transform_crs_dem = None
        if dem_layer and source.sourceCrs() != dem_layer.crs():
            transform_crs_dem = QgsCoordinateTransform(source.sourceCrs(), dem_layer.crs(), context.transformContext())

        def read_elevation(pt):
            if not dem_layer: return None
            p_sample = QgsPointXY(pt)
            if transform_crs_dem:
                try:
                    p_sample = transform_crs_dem.transform(p_sample)
                except: return None
            val, res = dem_layer.dataProvider().sample(p_sample, 1)
            if res and not math.isnan(val): return val
            return None

        waypoints_with_height = []
        base_altitude_start = None
        if dem_layer:
            for pt in waypoints:
                v = read_elevation(pt)
                if v is not None:
                    base_altitude_start = v
                    break
        if base_altitude_start is None: base_altitude_start = 0 

        for pt in waypoints:
            final_height = flight_altitude
            if dem_layer:
                v = read_elevation(pt)
                if v is not None:
                    final_height = int(round((v - base_altitude_start) + flight_altitude))
            waypoints_with_height.append((pt.x(), pt.y(), final_height))

        if sink_lines is not None:
            line_feat = QgsFeature(line_fields)
            line_feat.setGeometry(QgsGeometry.fromPolylineXY(waypoints))
            line_feat.setAttribute(0, 1)
            sink_lines.addFeature(line_feat, QgsFeatureSink.FastInsert)

        if sink_points is not None:
            total_pts = len(waypoints_with_height)
            for i, pt_data in enumerate(waypoints_with_height):
                pt_feat = QgsFeature(point_fields)
                pt_feat.setGeometry(QgsGeometry.fromPointXY(QgsPointXY(pt_data[0], pt_data[1])))
                lbl = "Start" if i == 0 else ("End" if i == total_pts - 1 else "")
                pt_feat.setAttributes([i, pt_data[2], flight_speed, "Smooth Flight", lbl])
                sink_points.addFeature(pt_feat, QgsFeatureSink.FastInsert)

        # ==========================================
        # ESTIMATED FLIGHT TIME & SAFETY CHECK
        # ==========================================
        dist_calc = QgsDistanceArea()
        dist_calc.setSourceCrs(source.sourceCrs(), context.transformContext())
        dist_calc.setEllipsoid(source.sourceCrs().ellipsoidAcronym())
        total_dist_meters = dist_calc.measureLength(QgsGeometry.fromPolylineXY(waypoints))
        
        base_time_sec = total_dist_meters / flight_speed
        num_curves = max(0, len(clipped_lines) - 1)
        
        # Calibrated Math (0.7s inertia penalty for DJI Mini 5 Pro)
        delay_per_curve = 2.0 + (flight_speed * 0.7)
        penalty_sec = num_curves * delay_per_curve
        total_time_sec = base_time_sec + penalty_sec
        total_time_min = total_time_sec / 60.0
        minutes = int(total_time_min)
        seconds = int(total_time_sec % 60)

        if total_time_min > MAX_FLIGHT_TIME:
            feedback.pushWarning(f"🚨 DANGER: Estimated flight time ({total_time_min:.1f} min) exceeds safe limit of {MAX_FLIGHT_TIME} min!")

        feedback.pushInfo("\n" + "="*45)
        feedback.pushInfo("📊 FLIGHT REPORT (FIELD ESTIMATE)")
        feedback.pushInfo(f"   Total Route Distance: {total_dist_meters:.2f} meters")
        feedback.pushInfo(f"   Transitions (Border curves): {num_curves}")
        feedback.pushInfo(f"   Cruising Speed: {flight_speed} m/s")
        feedback.pushInfo(f"   Estimated Flight Time: {minutes} min and {seconds} sec")
        feedback.pushInfo("="*45 + "\n")

        if kmz_path and kmz_path != 'TEMPORARY_OUTPUT':
            timestamp = int(time.time() * 1000)
            placemarks_xml = ""
            transform_wgs84 = None
            if source.sourceCrs().authid() != "EPSG:4326":
                wgs84_crs = QgsCoordinateReferenceSystem("EPSG:4326")
                transform_wgs84 = QgsCoordinateTransform(source.sourceCrs(), wgs84_crs, context.transformContext())

            for i, pt in enumerate(waypoints_with_height):
                if transform_wgs84:
                    try:
                        pt_geom = QgsGeometry.fromPointXY(QgsPointXY(pt[0], pt[1]))
                        pt_geom.transform(transform_wgs84)
                        lon, lat = pt_geom.asPoint().x(), pt_geom.asPoint().y()
                    except:
                        lon, lat = pt[0], pt[1]
                else:
                    lon, lat = pt[0], pt[1]
                h = pt[2]
                
                action_group_xml = f"""
        <wpml:actionGroup>
          <wpml:actionGroupId>1</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{i}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{i}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>parallel</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>reachPoint</wpml:actionTriggerType></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>1</wpml:actionId>
            <wpml:actionActuatorFunc>gimbalEvenlyRotate</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:gimbalPitchRotateAngle>-90</wpml:gimbalPitchRotateAngle>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""
                turn_mode = "toPointAndStopWithContinuityCurvature" if i == 0 else "toPointAndPassWithContinuityCurvature"
                placemarks_xml += f"""
      <Placemark>
        <Point><coordinates>{lon},{lat}</coordinates></Point>
        <wpml:index>{i}</wpml:index>
        <wpml:executeHeight>{h}</wpml:executeHeight>
        <wpml:waypointSpeed>{flight_speed}</wpml:waypointSpeed>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
          <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
          <wpml:waypointHeadingAngleEnable>0</wpml:waypointHeadingAngleEnable>
          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode>
        </wpml:waypointHeadingParam>
        <wpml:waypointTurnParam>
          <wpml:waypointTurnMode>{turn_mode}</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>0.2</wpml:waypointTurnDampingDist>
        </wpml:waypointTurnParam>
        <wpml:useStraightLine>0</wpml:useStraightLine>{action_group_xml}
      </Placemark>"""

            template_kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.2">
  <Document>
    <wpml:author>QGIS_GeoLogic</wpml:author><wpml:createTime>{timestamp}</wpml:createTime><wpml:updateTime>{timestamp}</wpml:updateTime>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode><wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost><wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:globalTransitionalSpeed>{flight_speed}</wpml:globalTransitionalSpeed>
      <wpml:droneInfo><wpml:droneEnumValue>68</wpml:droneEnumValue><wpml:droneSubEnumValue>0</wpml:droneSubEnumValue></wpml:droneInfo>
    </wpml:missionConfig>
  </Document>
</kml>"""
            waylines_wpml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.2">
  <Document>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode><wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost><wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:globalTransitionalSpeed>{flight_speed}</wpml:globalTransitionalSpeed>
      <wpml:droneInfo><wpml:droneEnumValue>68</wpml:droneEnumValue><wpml:droneSubEnumValue>0</wpml:droneSubEnumValue></wpml:droneInfo>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateId>0</wpml:templateId><wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>
      <wpml:waylineId>0</wpml:waylineId><wpml:distance>0</wpml:distance><wpml:duration>0</wpml:duration>
      <wpml:autoFlightSpeed>{flight_speed}</wpml:autoFlightSpeed>{placemarks_xml}
    </Folder>
  </Document>
</kml>"""
            temp_dir = tempfile.mkdtemp()
            wpmz_dir = os.path.join(temp_dir, "wpmz")
            os.makedirs(wpmz_dir)
            with open(os.path.join(wpmz_dir, "template.kml"), 'w', encoding='utf-8') as f: f.write(template_kml)
            with open(os.path.join(wpmz_dir, "waylines.wpml"), 'w', encoding='utf-8') as f: f.write(waylines_wpml)
            with zipfile.ZipFile(kmz_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
                zipf.write(os.path.join(wpmz_dir, "template.kml"), "wpmz/template.kml")
                zipf.write(os.path.join(wpmz_dir, "waylines.wpml"), "wpmz/waylines.wpml")

        line_details = context.layerToLoadOnCompletionDetails(dest_id_lines)
        if line_details:
            algo_instance.pp_lines = GeoLogicStylePostProcessor('lines')
            line_details.setPostProcessor(algo_instance.pp_lines)
        point_details = context.layerToLoadOnCompletionDetails(dest_id_points)
        if point_details:
            algo_instance.pp_points = GeoLogicStylePostProcessor('points')
            point_details.setPostProcessor(algo_instance.pp_points)

    except Exception as e:
        feedback.reportError(f"Critical Error: {str(e)}\n{traceback.format_exc()}")
    
    return results


# ==============================================================================
# TOOL 1: SIMPLE MODE (For everyday use / Express Planning)
# ==============================================================================
class DjiSimpleWaypointAlgorithm(QgsProcessingAlgorithm):
    INPUT = 'INPUT'
    DEM = 'DEM'
    ALTITUDE = 'ALTITUDE'
    SPEED = 'SPEED'
    OVERLAP_PERCENT = 'OVERLAP_PERCENT'
    OUTPUT_KMZ = 'OUTPUT_KMZ'
    OUTPUT_LINES = 'OUTPUT_LINES'
    OUTPUT_POINTS = 'OUTPUT_POINTS'

    def tr(self, string): return QCoreApplication.translate('Processing', string)
    def createInstance(self): return DjiSimpleWaypointAlgorithm()
    def name(self): return 'export_dji_wpml_simple'
    def displayName(self): return self.tr('Generate DJI Mission (Simple)')
    def group(self): return self.tr('GeoLogic Scripts')
    def groupId(self): return 'geologicscripts'
    def shortHelpString(self): return self.tr("Express mode for quick, safe flight planning. Auto-handles rotation, dynamic densification, and buffer safety.")

    def initAlgorithm(self, config=None):
        self.addParameter(QgsProcessingParameterFeatureSource(self.INPUT, self.tr('Area Polygon'), [QgsProcessing.TypeVectorPolygon]))
        self.addParameter(QgsProcessingParameterRasterLayer(self.DEM, self.tr('DEM (Terrain Following) - Optional'), optional=True))
        self.addParameter(QgsProcessingParameterNumber(self.ALTITUDE, self.tr('Flight Altitude (m)'), type=QgsProcessingParameterNumber.Integer, defaultValue=100))
        self.addParameter(QgsProcessingParameterNumber(self.SPEED, self.tr('Flight Speed (m/s)'), type=QgsProcessingParameterNumber.Double, defaultValue=9.0))
        self.addParameter(QgsProcessingParameterNumber(self.OVERLAP_PERCENT, self.tr('Lateral Overlap (%)'), type=QgsProcessingParameterNumber.Double, defaultValue=83.0))
        
        self.addParameter(QgsProcessingParameterFileDestination(self.OUTPUT_KMZ, self.tr('Save KMZ to:'), 'KMZ Files (*.kmz)', optional=True))
        self.addParameter(QgsProcessingParameterFeatureSink(self.OUTPUT_LINES, self.tr('Flight Route')))
        self.addParameter(QgsProcessingParameterFeatureSink(self.OUTPUT_POINTS, self.tr('Waypoints')))

    def processAlgorithm(self, parameters, context, feedback):
        source = self.parameterAsSource(parameters, self.INPUT, context)
        dem_layer = self.parameterAsRasterLayer(parameters, self.DEM, context)
        flight_altitude = self.parameterAsInt(parameters, self.ALTITUDE, context)
        flight_speed = self.parameterAsDouble(parameters, self.SPEED, context)
        overlap_percent = self.parameterAsDouble(parameters, self.OVERLAP_PERCENT, context)
        
        # Express Logic: Calculate Spacing, and Force Buffer to Spacing / 2.0
        footprint_width = flight_altitude * 1.5
        spacing_val = footprint_width * (1.0 - (overlap_percent / 100.0))
        buffer_val = spacing_val / 2.0
        dens_val = 10.0 # Tested and dynamically scaled by the engine
        
        feedback.pushInfo(f"ℹ️ Express Mode: Auto-spacing = {spacing_val:.2f}m. Auto-buffer = {buffer_val:.2f}m.")
        
        kmz_path = self.parameterAsFileOutput(parameters, self.OUTPUT_KMZ, context)
        
        point_fields = QgsFields()
        point_fields.append(QgsField("wp_id", QVariant.Int))
        point_fields.append(QgsField("height_m", QVariant.Int))
        point_fields.append(QgsField("speed_ms", QVariant.Double))
        point_fields.append(QgsField("action", QVariant.String))
        point_fields.append(QgsField("label", QVariant.String))

        line_fields = QgsFields()
        line_fields.append(QgsField("route_id", QVariant.Int))

        crs = source.sourceCrs() if source else None
        sink_lines, dest_id_lines = self.parameterAsSink(parameters, self.OUTPUT_LINES, context, line_fields, QgsWkbTypes.LineString, crs)
        sink_points, dest_id_points = self.parameterAsSink(parameters, self.OUTPUT_POINTS, context, point_fields, QgsWkbTypes.Point, crs)

        return process_flight_logic(self, context, feedback, source, dem_layer, flight_altitude, flight_speed, 
                                    spacing_val, buffer_val, dens_val, False, 0.0, kmz_path, 
                                    dest_id_lines, sink_lines, dest_id_points, sink_points, line_fields, point_fields, True)


# ==============================================================================
# TOOL 2: ADVANCED MODE (Full parameter control)
# ==============================================================================
class DjiAdvancedWaypointAlgorithm(QgsProcessingAlgorithm):
    INPUT = 'INPUT'
    DEM = 'DEM'
    ALTITUDE = 'ALTITUDE'
    SPEED = 'SPEED'
    
    USE_OVERLAP = 'USE_OVERLAP'
    OVERLAP_PERCENT = 'OVERLAP_PERCENT'
    SPACING = 'SPACING'
    
    USE_CUSTOM_ANGLE = 'USE_CUSTOM_ANGLE'
    CUSTOM_ANGLE = 'CUSTOM_ANGLE'
    
    BUFFER_AREA = 'BUFFER_AREA'
    DENSIFICATION = 'DENSIFICATION'
    OUTPUT_KMZ = 'OUTPUT_KMZ'
    OUTPUT_LINES = 'OUTPUT_LINES'
    OUTPUT_POINTS = 'OUTPUT_POINTS'

    def tr(self, string): return QCoreApplication.translate('Processing', string)
    def createInstance(self): return DjiAdvancedWaypointAlgorithm()
    def name(self): return 'export_dji_wpml_advanced'
    def displayName(self): return self.tr('Generate DJI Mission (Advanced)')
    def group(self): return self.tr('GeoLogic Scripts')
    def groupId(self): return 'geologicscripts'
    def shortHelpString(self): return self.tr("Maximum route optimization with manual azimuth control, automated overlap calculation, native CRS, and RC2 memory protection.")

    def initAlgorithm(self, config=None):
        self.addParameter(QgsProcessingParameterFeatureSource(self.INPUT, self.tr('Area Polygon'), [QgsProcessing.TypeVectorPolygon]))
        self.addParameter(QgsProcessingParameterRasterLayer(self.DEM, self.tr('DEM (Terrain Following) - Optional'), optional=True))
        self.addParameter(QgsProcessingParameterNumber(self.ALTITUDE, self.tr('Flight Altitude (m)'), type=QgsProcessingParameterNumber.Integer, defaultValue=100))
        self.addParameter(QgsProcessingParameterNumber(self.SPEED, self.tr('Flight Speed (m/s)'), type=QgsProcessingParameterNumber.Double, defaultValue=9.0))
        
        self.addParameter(QgsProcessingParameterBoolean(self.USE_CUSTOM_ANGLE, self.tr('Use Custom Flight Line Azimuth'), defaultValue=False))
        self.addParameter(QgsProcessingParameterNumber(self.CUSTOM_ANGLE, self.tr('Custom Azimuth (0=North, 90=East) [If checked]'), type=QgsProcessingParameterNumber.Double, defaultValue=0.0))
        
        self.addParameter(QgsProcessingParameterBoolean(self.USE_OVERLAP, self.tr('Calculate spacing using Overlap %'), defaultValue=True))
        self.addParameter(QgsProcessingParameterNumber(self.OVERLAP_PERCENT, self.tr('Lateral Overlap (%)'), type=QgsProcessingParameterNumber.Double, defaultValue=83.0))
        self.addParameter(QgsProcessingParameterNumber(self.SPACING, self.tr('Manual Line Spacing (m) [If checkbox is unchecked]'), type=QgsProcessingParameterNumber.Double, defaultValue=30.0))
        
        self.addParameter(QgsProcessingParameterNumber(self.BUFFER_AREA, self.tr('Boundary Buffer (m)'), type=QgsProcessingParameterNumber.Double, defaultValue=15.0))
        self.addParameter(QgsProcessingParameterNumber(self.DENSIFICATION, self.tr('DEM Densification (m)'), type=QgsProcessingParameterNumber.Double, defaultValue=10.0)) 
        
        self.addParameter(QgsProcessingParameterFileDestination(self.OUTPUT_KMZ, self.tr('Save KMZ to:'), 'KMZ Files (*.kmz)', optional=True))
        self.addParameter(QgsProcessingParameterFeatureSink(self.OUTPUT_LINES, self.tr('Flight Route')))
        self.addParameter(QgsProcessingParameterFeatureSink(self.OUTPUT_POINTS, self.tr('Waypoints')))

    def processAlgorithm(self, parameters, context, feedback):
        source = self.parameterAsSource(parameters, self.INPUT, context)
        dem_layer = self.parameterAsRasterLayer(parameters, self.DEM, context)
        flight_altitude = self.parameterAsInt(parameters, self.ALTITUDE, context)
        flight_speed = self.parameterAsDouble(parameters, self.SPEED, context)
        
        use_overlap = self.parameterAsBool(parameters, self.USE_OVERLAP, context)
        if use_overlap:
            overlap_percent = self.parameterAsDouble(parameters, self.OVERLAP_PERCENT, context)
            footprint_width = flight_altitude * 1.5
            spacing_val = footprint_width * (1.0 - (overlap_percent / 100.0))
            feedback.pushInfo(f"ℹ️ Auto-Spacing: Footprint width is {footprint_width}m. Calculating {overlap_percent}% overlap -> Line Spacing = {spacing_val:.2f}m")
        else:
            spacing_val = self.parameterAsDouble(parameters, self.SPACING, context)
            if spacing_val <= 1.0: spacing_val = 30.0 
            feedback.pushInfo(f"ℹ️ Manual Spacing Mode: Using fixed {spacing_val}m line spacing.")
            
        use_custom_angle = self.parameterAsBool(parameters, self.USE_CUSTOM_ANGLE, context)
        azimuth_val = self.parameterAsDouble(parameters, self.CUSTOM_ANGLE, context) if use_custom_angle else 0.0
        
        buffer_val = self.parameterAsDouble(parameters, self.BUFFER_AREA, context)
        dens_val = self.parameterAsDouble(parameters, self.DENSIFICATION, context)
        if dens_val <= 1.0: dens_val = 10.0 
        
        kmz_path = self.parameterAsFileOutput(parameters, self.OUTPUT_KMZ, context)

        point_fields = QgsFields()
        point_fields.append(QgsField("wp_id", QVariant.Int))
        point_fields.append(QgsField("height_m", QVariant.Int))
        point_fields.append(QgsField("speed_ms", QVariant.Double))
        point_fields.append(QgsField("action", QVariant.String))
        point_fields.append(QgsField("label", QVariant.String))

        line_fields = QgsFields()
        line_fields.append(QgsField("route_id", QVariant.Int))

        crs = source.sourceCrs() if source else None
        sink_lines, dest_id_lines = self.parameterAsSink(parameters, self.OUTPUT_LINES, context, line_fields, QgsWkbTypes.LineString, crs)
        sink_points, dest_id_points = self.parameterAsSink(parameters, self.OUTPUT_POINTS, context, point_fields, QgsWkbTypes.Point, crs)

        return process_flight_logic(self, context, feedback, source, dem_layer, flight_altitude, flight_speed, 
                                    spacing_val, buffer_val, dens_val, use_custom_angle, azimuth_val, kmz_path, 
                                    dest_id_lines, sink_lines, dest_id_points, sink_points, line_fields, point_fields, False)