# GeoLogic Flight Planner 🚁🗺️

**A PyQGIS plugin for automated DJI drone mission planning. Generates native KMZ/WPML files featuring DEM terrain following, optimal route orientation, precise flight time estimation, and smart waypoint densification to safely respect DJI hardware limits.**

---

## 📌 Overview

The **GeoLogic Flight Planner** is a custom QGIS processing plugin built to bridge the gap between GIS environments and standalone drone operations. Many consumer-tier DJI drones lack native grid mission planning capabilities within the DJI Fly application, forcing operators to rely on expensive, restrictive, or cloud-dependent third-party software. Furthermore, modern DJI controllers require a highly specific, packaged XML-based format (`.kmz` containing `.wpml` schemas) to accept offline routes.

This plugin allows professionals to design highly optimized, terrain-aware flight plans completely offline inside QGIS, exporting directly to a DJI-native compliant format.

---

## ⚙️ Core Algorithmic Features

### 1. Optimal Angle Sweep (Grid Orientation)
To maximize battery efficiency, the algorithm minimizes the number of turns (border curves). It evaluates the input polygon's geometry across a 180-degree rotation space using an oriented bounding box approach. It automatically selects the azimuth that yields the shortest bounding height, resulting in longer straight legs and fewer battery-draining transitions.

### 2. Terrain Following (DEM Integration)
By sampling a local Digital Elevation Model (DEM) raster layer, the plugin extracts elevation data at every waypoint. It establishes a relative base altitude from the launch point and dynamically adjusts the drone's flight height along the grid paths, ensuring a constant Ground Sampling Distance (GSD) and safer clearance over rugged topographies.

### 3. DJI RC2 Hardware Memory Protection (Smart Densification)
The DJI RC2 controller has a strict hardware limitation that can cause application instability or flight execution crashes if a mission file exceeds **200 waypoints**. 
* The plugin features an **Internal Brute-Force Simulator** that counts exact points (base line edges + intermediate terrain points) before writing the file.
* If a detailed DEM profile forces the waypoint count past 200, the **Smart Densification Engine** automatically steps up the sampling distance iteratively (from 10 meters up to a 40-meter hard cap) until the file safely fits within the controller's memory. It triggers a clear dashboard warning if the route remains too massive.

### 4. Kinematic Flight Time Estimation & Autonomy Safeties
Traditional planners divide total distance by cruise speed, neglecting multi-axis drone deceleration. This plugin applies a calibrated kinematic penalty model:
**Delay per turn** = 2.0s + (*flight_speed* × 0.7s)
This accounts for drone inertia during border transitions. If the cumulative flight time exceeds **35 minutes**—the typical safe operational threshold for commercial drone batteries—the plugin throws a critical `🚨 DANGER` alert in the QGIS log panel to prevent unsafe field deployments.

### 5. Native WPML Architecture Compliance
Instead of exporting generic KML points, the plugin auto-generates a structured, compressed ZIP stream renamed as a `.kmz`. Inside, it dynamically writes standard DJI packages:
* `template.kml`: Sets global mission configurations, safe execution heights, failsafe behaviors (`goHome`, `exitOnRCLost`), and hardware payload indices.
* `waylines.wpml`: Structures individual `<Placemark>` nodes detailing unique coordinates, localized speeds, continuous smooth turn indices (`toPointAndPassWithContinuityCurvature`), and parallel gimbal pitch actions (`gimbalEvenlyRotate` at -90° nadir).

---

## 🛠️ Included Tools

The plugin populates the QGIS Processing Toolbox with two distinct modules under the **GeoLogic Flight Tools** provider:

### 💡 1. Generate DJI Mission (Simple)
*Designed for express, error-free daily planning.* * **Automated Photogrammetry Math:** Automatically calculates optimal line spacing based on the drone's optical camera footprint ($Width  pprox Altitude 	imes 1.5$) matching your requested **Lateral Overlap %** (defaulting to an optimal 83%).
* **Auto-Buffer:** Automatically shrinks the flight area inward by a safety buffer equal to half the track spacing (`spacing / 2`), protecting the drone from boundary overshoots.
* **Smart Defaulting:** Automatically scales the elevation densification starting at a tight 10-meter resolution.

### ⚙️ 2. Generate DJI Mission (Advanced)
*Designed for complex topographies or custom mapping scenarios demanding granular control.*
* **Manual Azimuth Overrides:** Turn off automatic orientation to align flight lines with linear structural geology features, custom survey lines, or specific lighting angles.
* **Granular Controls:** Manually dictate exact line spacing (meters), outer boundary buffer distances, and baseline DEM sampling intervals.

---

## 📥 Installation Guide

Since this plugin is currently shared as an offline package, you can easily install it manually inside QGIS using the packaged zip format:

1. Click the green **Code** button at the top right of this GitHub repository and select **Download ZIP**.
2. Open **QGIS** (v3.0 or higher supported; checked for upcoming v4.0 metadata specifications).
3. In the top menu, navigate to **Plugins** > **Manage and Install Plugins...** (*Complementos > Gerenciar e Instalar Complementos...*).
4. In the left-hand sidebar, click on the **Install from ZIP** tab.
5. Click the ellipsis button (`...`), locate the downloaded repository ZIP file on your computer, and select it.
6. Click **Install Plugin**. A success banner will confirm the engine is successfully integrated into your local QGIS registry.

---

## 🚀 Execution Workflow

1. Open the **Processing Toolbox** panel in QGIS (Gear icon).
2. Expand **GeoLogic Scripts** > **GeoLogic Flight Tools** and double-click either the **Simple** or **Advanced** tool.
3. Configure your vector input layer (**Area Polygon**) and your raster elevation layer (**DEM**).
4. Define your operational cruise metrics (Flight Altitude and Speed).
5. Specify your export target directory in the **Save KMZ to** file parameter field.
6. Click **Run**.
7. View full telemetria metrics, waypoint counts, and flight duration warnings directly in the QGIS Log tab. 
8. The tool will automatically style and add **GeoLogic - Flight Route** lines and **GeoLogic - Waypoints** layers to your QGIS map canvas using explicit custom symbology (Triangles for start points, Squares for endpoints, and clean labeled point buffers).
9. Copy the generated `.kmz` file directly to your DJI Controller's internal memory/SD card and import the task natively via the Pilot/Fly app interface.

---

## 👨‍💻 Developer & Project Context

* **Author:** Victor Moreira Naves Ribeiro
* **Background:** Geologist & Systems Analysis and Development (ADS) Technologist.
* **Academic Focus:** Master's Researcher in Geochronology and Isotopic Geology at the University of Brasília (UnB).

*This core processing engine was architected under the **"Vibe Coding"** philosophy—seamlessly pairing domain-specific geospatial engineering logic with Advanced Artificial Intelligence to accelerate code optimization, framework parsing, and hardware compliance.*

Connect or follow project logs via LinkedIn:  
🔗 [Victor Moreira Naves Ribeiro on LinkedIn](https://www.linkedin.com/in/victor-ribeiro-50315821a/)
