import numpy as np
import math
import cv2
import os

class KittiVisualizer:
    def __init__(self, lidar_dir, oxts_dir, bev_size=800, res=0.1):
        self.lidar_dir = lidar_dir
        self.oxts_dir = oxts_dir
        self.bev_size = bev_size
        self.res = res
        self.files = sorted([f for f in os.listdir(lidar_dir) if f.endswith('.bin')])

    def get_oxts_data(self, filename):
        """ Extract speed in m/s from oxts file """
        oxts_path = os.path.join(self.oxts_dir, filename.replace('.bin', '.txt'))
        with open(oxts_path, 'r') as f:
            data = f.readline().split()
            speed = float(data[8]) * 3.6  # m/s to km/h
            return speed

    def lidar_to_bev(self, lidar_path):
        """ Transform point cloud into 4 maps Color, Height, Intensity, Density) """
        pts = np.fromfile(lidar_path, dtype=np.float32).reshape(-1, 4)
        
        # Coordinates
        x, y, z, intensity = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]

        # Pixel indices (vehicle centered)
        px = (self.bev_size // 2 + y / self.res).astype(int)
        py = (self.bev_size // 2 - x / self.res).astype(int)

        # Mask : image borders + ground filtering (z > -1.5m)
        mask = (px >= 0) & (px < self.bev_size) & (py >= 0) & (py < self.bev_size)
        px, py, z_m, i_m = px[mask], py[mask], z[mask], intensity[mask]

        # Maps initialization
        bev_height = np.full((self.bev_size, self.bev_size), -10.0, dtype=np.float32)
        bev_intensity = np.zeros((self.bev_size, self.bev_size), dtype=np.float32)
        bev_density = np.zeros((self.bev_size, self.bev_size), dtype=np.float32)

        # Optimized filling
        np.maximum.at(bev_height, (py, px), z_m)
        np.maximum.at(bev_intensity, (py, px), i_m)
        np.add.at(bev_density, (py, px), 1)

        return self.normalize_and_color(bev_height, bev_intensity, bev_density)

    def normalize_and_color(self, h, i, d):
        """ Normalize and apply colormaps """
        def norm(arr, vmin, vmax):
            return (np.clip((arr - vmin) / (vmax - vmin), 0, 1) * 255).astype(np.uint8)

        # Valid points mask for colorization
        valid = h > -10.0
        
        # Separate image for height, intensity and density
        h_img = cv2.applyColorMap(norm(h, -1.5, 2.5), cv2.COLORMAP_JET)
        i_img = cv2.applyColorMap(norm(i, 0, 1), cv2.COLORMAP_HOT)
        d_img = cv2.applyColorMap(norm(d, 0, 5), cv2.COLORMAP_VIRIDIS)
        
        # Main image bev 
        bev_main = np.zeros((self.bev_size, self.bev_size, 3), dtype=np.uint8)
        bev_main[valid] = h_img[valid]

        return bev_main, h_img, i_img, d_img

    def run(self, output_name="results/kitti_bev_final.mp4"):
        # Video initialization
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_name, fourcc, 10, (800, 800))

        for f in self.files:
            speed = self.get_oxts_data(f)
            main, h, i, d = self.lidar_to_bev(os.path.join(self.lidar_dir, f))

            # 2x2 to visualize every fiels
            top = np.hstack((main, h))
            bot = np.hstack((i, d))
            grid = np.vstack((top, bot))
            grid_resised = cv2.resize(grid, (800, 800))

            # Telemetry Overlay
            overlay = grid_resised.copy()
            cv2.rectangle(overlay, (5, 5), (280, 80), (0,0,0), -1)
            grid_resised = cv2.addWeighted(overlay, 0.6, grid_resised, 0.4, 0)
            
            cv2.putText(grid_resised, f"Speed: {speed:.1f} km/h", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.imshow("KITTI Autonomous Dashboard", grid_resised)
            out.write(grid_resised)

            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

        out.release()
        cv2.destroyAllWindows()

# --- EXECUTION ---
# Adjust according to your own path
LiDAR_DIR = "data/velodyne_points/data/"
OXTS_DIR = "data/oxts/data/"
output_dir = "results/kitti_bev.mp4"

viz = KittiVisualizer(LiDAR_DIR, OXTS_DIR)
viz.run(output_dir)