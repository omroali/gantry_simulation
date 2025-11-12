#!/usr/bin/env python3
"""
Satellite Tile Downloader for ROS

This script downloads and caches satellite imagery tiles from various sources.
It supports multi-threaded downloading and can stream tiles based on GPS location.

Usage:
    rosrun gantry_simulation satellite_tile_downloader.py

Parameters:
    ~tile_url (str): URL template for tile source (default: Esri World Imagery)
    ~zoom_level (int): Zoom level for tiles (default: 18)
    ~api_key (str): API key if required by tile source (optional)
    ~num_workers (int): Number of download threads (default: 4)
    ~gps_topic (str): GPS topic to subscribe to (default: /gps/fix)
"""

import rospy
import requests
import os
from pathlib import Path
from PIL import Image
from io import BytesIO
import threading
from queue import Queue, Empty
import logging
from math import log2, tan, cos, pi, atan, sinh
from sensor_msgs.msg import NavSatFix

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class SatelliteTileDownloader:
    """Download and cache satellite imagery tiles"""

    def __init__(self):
        """Initialize the satellite tile downloader"""
        rospy.init_node("satellite_tile_downloader", anonymous=False)

        # Configuration from parameters
        self.cache_dir = Path.home() / ".cache" / "rviz_satellite"
        self.cache_dir.mkdir(parents=True, exist_ok=True)

        self.tile_url = rospy.get_param(
            "~tile_url",
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        )

        self.api_key = rospy.get_param("~api_key", "")
        self.zoom_level = rospy.get_param("~zoom_level", 18)
        self.num_workers = rospy.get_param("~num_workers", 4)
        self.gps_topic = rospy.get_param("~gps_topic", "/gps/fix")

        # Download queue for threaded downloads
        self.download_queue = Queue()
        self.worker_threads = []
        self.downloaded_tiles = set()
        self.failed_tiles = set()

        # Threading control
        self.shutdown_flag = False
        self.lock = threading.Lock()

        # Start worker threads
        for i in range(self.num_workers):
            t = threading.Thread(
                target=self._worker, daemon=True, name=f"TileWorker-{i}"
            )
            t.start()
            self.worker_threads.append(t)

        # Subscribe to GPS updates
        rospy.Subscriber(self.gps_topic, NavSatFix, self._gps_callback)

        rospy.loginfo(f"Satellite Tile Downloader initialized")
        rospy.loginfo(f"Cache directory: {self.cache_dir}")
        rospy.loginfo(f"Tile source: {self.tile_url}")
        rospy.loginfo(f"Zoom level: {self.zoom_level}")
        rospy.loginfo(f"Worker threads: {self.num_workers}")

    def _gps_callback(self, msg):
        """Callback for GPS updates - download tiles around current position"""
        try:
            lat = msg.latitude
            lon = msg.longitude

            # Download tiles around GPS position (3x3 grid)
            self._download_tiles_around_position(lat, lon, radius=1)
        except Exception as e:
            rospy.logerr(f"Error in GPS callback: {e}")

    def _lat_lon_to_tile(self, lat, lon, z):
        """Convert latitude/longitude to tile coordinates"""
        try:
            n = 2.0**z
            x = int((lon + 180.0) / 360.0 * n)

            lat_rad = lat * pi / 180.0
            y = int((1.0 - log2(tan(lat_rad) + 1.0 / cos(lat_rad)) / pi) / 2.0 * n)

            return x, y
        except Exception as e:
            rospy.logerr(f"Error converting lat/lon to tile: {e}")
            return None, None

    def _get_tile_url(self, z, x, y):
        """Generate tile URL with proper formatting"""
        url = self.tile_url.format(z=z, x=x, y=y)
        if self.api_key and "{key}" in url:
            url = url.replace("{key}", self.api_key)
        elif self.api_key and "key=" not in url and "?" in url:
            url += f"&key={self.api_key}"
        elif self.api_key and "?" not in url:
            url += f"?key={self.api_key}"
        return url

    def _get_cache_path(self, z, x, y):
        """Get local cache path for tile"""
        tile_dir = self.cache_dir / f"z{z}"
        tile_dir.mkdir(parents=True, exist_ok=True)
        return tile_dir / f"{x}_{y}.png"

    def _worker(self):
        """Worker thread that processes download queue"""
        thread_name = threading.current_thread().name
        while not self.shutdown_flag:
            try:
                z, x, y = self.download_queue.get(timeout=1)
                self.download_tile(z, x, y)
            except Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Worker error: {e}")

    def download_tile(self, z, x, y, force=False, timeout=10):
        """Download a single tile with caching"""
        cache_path = self._get_cache_path(z, x, y)

        # Check if already cached
        if cache_path.exists() and not force:
            with self.lock:
                self.downloaded_tiles.add((z, x, y))
            return True

        # Avoid re-downloading failed tiles
        with self.lock:
            if (z, x, y) in self.failed_tiles and not force:
                return False

        try:
            url = self._get_tile_url(z, x, y)
            response = requests.get(url, timeout=timeout)

            if response.status_code == 200:
                # Validate it's an image
                try:
                    img = Image.open(BytesIO(response.content))
                    img.verify()
                except Exception as e:
                    rospy.logwarn(f"Invalid image for {z}/{x}/{y}: {e}")
                    with self.lock:
                        self.failed_tiles.add((z, x, y))
                    return False

                # Save to cache
                with open(cache_path, "wb") as f:
                    f.write(response.content)

                with self.lock:
                    self.downloaded_tiles.add((z, x, y))

                rospy.logdebug(f"Downloaded tile {z}/{x}/{y}")
                return True
            else:
                rospy.logwarn(
                    f"Failed to download {z}/{x}/{y}: HTTP {response.status_code}"
                )
                with self.lock:
                    self.failed_tiles.add((z, x, y))
                return False

        except requests.exceptions.Timeout:
            rospy.logwarn(f"Timeout downloading {z}/{x}/{y}")
            return False
        except requests.exceptions.ConnectionError:
            rospy.logwarn(f"Connection error downloading {z}/{x}/{y}")
            return False
        except Exception as e:
            rospy.logerr(f"Error downloading tile {z}/{x}/{y}: {e}")
            return False

    def _download_tiles_around_position(self, lat, lon, radius=1):
        """Download tiles in a square around a position"""
        z = self.zoom_level

        center_x, center_y = self._lat_lon_to_tile(lat, lon, z)
        if center_x is None or center_y is None:
            return

        # Download tiles in a square pattern around center
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                x = center_x + dx
                y = center_y + dy

                tile_key = (z, x, y)
                with self.lock:
                    if (
                        tile_key not in self.downloaded_tiles
                        and tile_key not in self.failed_tiles
                    ):
                        self.download_queue.put(tile_key)

    def download_tiles_for_region(self, lat_min, lat_max, lon_min, lon_max):
        """Download all tiles for a geographic region"""
        z = self.zoom_level

        x_min, y_max = self._lat_lon_to_tile(lat_min, lon_min, z)
        x_max, y_min = self._lat_lon_to_tile(lat_max, lon_max, z)

        if x_min is None or x_max is None:
            rospy.logerr("Invalid coordinates for region")
            return 0

        rospy.loginfo(
            f"Downloading tiles for region: "
            f"({lat_min:.4f}, {lon_min:.4f}) to ({lat_max:.4f}, {lon_max:.4f})"
        )

        count = 0
        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                tile_key = (z, x, y)
                with self.lock:
                    if tile_key not in self.downloaded_tiles:
                        self.download_queue.put(tile_key)
                        count += 1

        rospy.loginfo(f"Queued {count} tiles for download")
        return count

    def get_cache_size(self):
        """Get total cache size in MB"""
        total_size = sum(
            f.stat().st_size for f in self.cache_dir.rglob("*") if f.is_file()
        )
        return total_size / (1024 * 1024)

    def get_download_stats(self):
        """Get download statistics"""
        with self.lock:
            return {
                "downloaded": len(self.downloaded_tiles),
                "failed": len(self.failed_tiles),
                "queued": self.download_queue.qsize(),
                "cache_size_mb": self.get_cache_size(),
            }

    def clear_cache(self):
        """Clear all cached tiles"""
        import shutil

        try:
            shutil.rmtree(self.cache_dir)
            self.cache_dir.mkdir(parents=True, exist_ok=True)
            with self.lock:
                self.downloaded_tiles.clear()
                self.failed_tiles.clear()
            rospy.loginfo("Cache cleared")
        except Exception as e:
            rospy.logerr(f"Error clearing cache: {e}")

    def run(self):
        """Run the downloader node"""
        rate = rospy.Rate(1)  # Publish stats at 1 Hz

        while not rospy.is_shutdown():
            try:
                stats = self.get_download_stats()
                rospy.loginfo(
                    f"Tiles: Downloaded={stats['downloaded']}, "
                    f"Failed={stats['failed']}, "
                    f"Queued={stats['queued']}, "
                    f"Cache={stats['cache_size_mb']:.1f}MB"
                )
                rate.sleep()
            except Exception as e:
                rospy.logerr(f"Error in main loop: {e}")
                rate.sleep()

    def shutdown(self):
        """Shutdown the downloader"""
        rospy.loginfo("Shutting down satellite tile downloader")
        self.shutdown_flag = True
        for t in self.worker_threads:
            t.join(timeout=5)
        rospy.loginfo("Downloader shut down complete")


def main():
    """Main entry point"""
    downloader = SatelliteTileDownloader()

    # Register shutdown hook
    rospy.on_shutdown(downloader.shutdown)

    try:
        downloader.run()
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")
        downloader.shutdown()


if __name__ == "__main__":
    main()
