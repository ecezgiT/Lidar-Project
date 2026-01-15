# LIDAR Data Processor & Visualizer

A C-based application designed to process 2D LIDAR point cloud data. This project parses raw sensor data (TOML format), converts polar coordinates to Cartesian, and applies the **RANSAC (Random Sample Consensus)** algorithm to detect environmental features like walls and obstacles.

## Key Features
* **Data Parsing:** Custom parser for TOML configuration files using `libcurl` for data retrieval.
* **Algorithm Implementation:** Manual implementation of the RANSAC algorithm to identify lines in noisy datasets.
* **Optimization:** Includes filtering logic to remove redundant lines based on slope and intersection angles.
* **Visualization:** Renders processed data and detected lines using the **Allegro 5** graphics library.

## Tech Stack
* **Language:** C
* **Libraries:** Allegro 5, libcurl
* **Tools:** Visual Studio
