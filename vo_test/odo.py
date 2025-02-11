# visual odometry without intrinsic camera parameter
# parameters are aproximated


import os
import numpy as np
import cv2
from tqdm import tqdm
import logging
from lib.visualization import plotting
from lib.visualization.video import play_trip

class VisualOdometry:
    def __init__(self, data_dir):
        self.images = self._load_images(os.path.join(data_dir, "output"))
        self.orb = cv2.ORB_create(3000)

        # FLANN-based matcher setup
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=500)
        self.flann = cv2.FlannBasedMatcher(indexParams=index_params, searchParams=search_params)
        
        self.logger = logging.getLogger('VisualOdometry')
        self.logger.setLevel(logging.INFO)

    @staticmethod
    def _load_images(filepath):
        """Load grayscale images from a given directory path."""
        image_paths = [os.path.join(filepath, file) for file in sorted(os.listdir(filepath))]
        return [cv2.imread(path, cv2.IMREAD_GRAYSCALE) for path in image_paths]

    @staticmethod
    def _form_transformation_matrix(R, t):
        """Form a 4x4 transformation matrix from R and t."""
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.squeeze()
        return T

    def get_matches(self, i):
        """Find feature matches between two consecutive frames."""
        kp1, des1 = self.orb.detectAndCompute(self.images[i - 1], None)
        kp2, des2 = self.orb.detectAndCompute(self.images[i], None)
        
        matches = self.flann.knnMatch(des1, des2, k=2)
        good_matches = [m for m, n in matches if m.distance < 0.8 * n.distance]

        if len(good_matches) < 8:
            self.logger.warning(f"Not enough good matches: {len(good_matches)}")
            return None, None

        q1 = np.float32([kp1[m.queryIdx].pt for m in good_matches])
        q2 = np.float32([kp2[m.trainIdx].pt for m in good_matches])
        return q1, q2

    def get_pose(self, q1, q2):
        """Estimate pose from Fundamental Matrix without camera parameters."""
        F, mask = cv2.findFundamentalMat(q1, q2, cv2.FM_RANSAC)
        
        if F is None or np.sum(mask) < 8:
            self.logger.warning("Fundamental matrix estimation failed or insufficient inliers.")
            return None

        # Assume a focal length for an approximate Essential Matrix
        focal_length = 1.0  # Arbitrary since there's no scale
        E = np.array([[F[0, 0] * focal_length, F[0, 1] * focal_length, F[0, 2]],
                      [F[1, 0] * focal_length, F[1, 1] * focal_length, F[1, 2]],
                      [F[2, 0], F[2, 1], F[2, 2]]])

        # Decompose Essential Matrix to get R and t
        R, t = self.decompose_essential_matrix(E, q1[mask.ravel() == 1], q2[mask.ravel() == 1])

        if R is None or t is None:
            self.logger.warning("Failed to retrieve valid R and t from Essential Matrix.")
            return None

        return self._form_transformation_matrix(R, t)

    def decompose_essential_matrix(self, E, q1, q2):
        """Decompose the essential matrix and select the correct R and t."""
        R1, R2, t = cv2.decomposeEssentialMat(E)
        candidates = [(R1, t), (R1, -t), (R2, t), (R2, -t)]
        
        best_pair, max_positive_z, scale = None, 0, None
        
        for R, t in candidates:
            positive_z = self._calculate_positive_z(R, t, q1, q2)
            if positive_z > max_positive_z:
                max_positive_z = positive_z
                best_pair = (R, t)

        if best_pair:
            return best_pair
        return None, None

    def _calculate_positive_z(self, R, t, q1, q2):
        """Count the number of positive depth values to determine the best motion hypothesis."""
        positive_z_count = np.sum((q1[:, 1] > 0) & (q2[:, 1] > 0))
        return positive_z_count

    def process_frames(self):
        """Process all frames and compute the visual odometry."""
        estimated_path = []
        cur_pose = np.eye(4)
        
        for i in tqdm(range(1, len(self.images))):
            try:
                q1, q2 = self.get_matches(i)
                if q1 is None or q2 is None:
                    continue
            except:
                continue

            transf = self.get_pose(q1, q2)
            if transf is None or np.isnan(transf).any() or np.isinf(transf).any():
                continue

            cur_pose = cur_pose @ np.linalg.inv(transf)
            estimated_path.append((cur_pose[0, 3], cur_pose[2, 3]))

        return estimated_path

def main():
    data_dir = "amb_sequences"
    vo = VisualOdometry(data_dir)

    play_trip(vo.images)  # Comment out to not play the trip

    estimated_path = vo.process_frames()
    
    if estimated_path:
        plotting.visualize_paths2(estimated_path, "Visual Odometry without Camera Parameters", file_out=os.path.basename(data_dir) + "no_intrinsics.html")
        np.savetxt('estimated_path_no_intrinsics.csv', estimated_path, delimiter=',')
    else:
        print("No path was estimated.")

if __name__ == "__main__":
    main()