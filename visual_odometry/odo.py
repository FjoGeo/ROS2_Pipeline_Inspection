import os
import numpy as np
import cv2
from tqdm import tqdm
import logging
from lib.visualization import plotting

class VisualOdometry:
    def __init__(self, data_dir):
        self.K = np.array([
            [918.6, 0.0, 641.6],
            [0.0, 918.9, 379.2],
            [0.0, 0.0, 1.0]
        ])

        self.P = np.array([
            [918.6, 0.0, 641.6, 0.0],
            [0.0, 918.9, 379.2, 0.0],
            [0.0, 0.0, 1.0, 0.0]
        ])
        
        self.images = self._load_images(os.path.join(data_dir, "image_r"))
        # self.images = self._load_images(os.path.join(data_dir, "image_l"))
        self.orb = cv2.ORB_create(3000)
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=500)
        self.flann = cv2.FlannBasedMatcher(indexParams=index_params, searchParams=search_params)
        self.logger = logging.getLogger('VisualOdometry')
        self.logger.setLevel(logging.INFO)

    @staticmethod
    def _load_images(filepath):
        """Load images from a given directory path."""
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
        """Estimate pose from point correspondences."""
        E, mask = cv2.findEssentialMat(q1, q2, self.K, method=cv2.RANSAC, threshold=1, prob=0.999)
        
        if E is None or np.sum(mask) < 8:
            self.logger.warning("Essential matrix decomposition failed or insufficient inliers.")
            return None

        R, t = self.decompose_essential_matrix(E, q1[mask.ravel() == 1], q2[mask.ravel() == 1])

        if R is None or t is None:
            self.logger.warning("Failed to retrieve valid R and t from essential matrix.")
            return None

        return self._form_transformation_matrix(R, t)

    def decompose_essential_matrix(self, E, q1, q2):
        """Decompose the essential matrix and select the correct R and t."""
        R1, R2, t = cv2.decomposeEssentialMat(E)
        candidates = [(R1, t), (R1, -t), (R2, t), (R2, -t)]
        
        best_pair, max_positive_z, scale = None, 0, None
        
        for R, t in candidates:
            positive_z, scale_factor = self._calculate_positive_z(R, t, q1, q2)
            if positive_z > max_positive_z:
                max_positive_z = positive_z
                best_pair = (R, t)
                scale = scale_factor

        if best_pair:
            R, t = best_pair
            t *= scale
            return R, t
        return None, None

    def _calculate_positive_z(self, R, t, q1, q2):
        """Calculate positive Z coordinates and relative scale."""
        T = self._form_transformation_matrix(R, t)
        P_new = self.K @ T[:3, :]
        
        hom_Q1 = cv2.triangulatePoints(self.P, P_new, q1.T, q2.T)
        hom_Q2 = T @ hom_Q1
        
        uhom_Q1 = hom_Q1[:3] / hom_Q1[3]
        uhom_Q2 = hom_Q2[:3] / hom_Q2[3]

        positive_z_count = np.sum((uhom_Q1[2] > 0) & (uhom_Q2[2] > 0))
        scale = np.mean(np.linalg.norm(np.diff(uhom_Q1.T, axis=0), axis=1) /
                        np.linalg.norm(np.diff(uhom_Q2.T, axis=0), axis=1))
        return positive_z_count, scale

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
    estimated_path = vo.process_frames()
    
    if estimated_path:
        # plotting.visualize_paths2(estimated_path, "Visual Odometry Single", file_out=os.path.basename(data_dir) + "single.html")
        np.savetxt('estimated_path_right.csv', estimated_path, delimiter=',')
    else:
        print("No path was estimated.")

if __name__ == "__main__":
    main()
