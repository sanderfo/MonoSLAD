import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import matplotlib.pyplot as plt
import cv2
import numpy as np
import pyquaternion as q
import time
import threading

images = []
depth_images = []
poses = []
with open("./test/transforms.txt", "r") as transforms:
    for i, line in enumerate(transforms):
        if i % 2 == 0:
            filename = "." + str(line.split("AutonomousLanding")[1])[:-1] 
            images.append(cv2.imread(filename + ".png", cv2.IMREAD_GRAYSCALE))
            if ((i / 2) - 1) % 2 == 0:
                depth_images.append(cv2.imread(filename + "depth.png"))

            
        else:
            words = line.split()
            transform = np.array(words, dtype = np.float64)
            poses.append(transform)
            

def quat_dist(q1, q2):
        return np.min((np.linalg.norm(q1 - q2), np.linalg.norm(q1 + q2)))

class PlaneSweepEpisode:

    def __init__(self, poses, images, depth_image):
        self.poses = poses
        self.images = images
        self.depth_image = depth_image

    def compute_roatation_distances(self):
        self.rotation_distances = []
        self.rotation_distances.append(
            quat_dist(self.poses[0][:4], self.poses[1][:4])
            )
        #self.rotation_distances.append(
        #    quat_dist(self.poses[0][:4], self.poses[2][:4])
        #    )
        #self.rotation_distances.append(
        #    quat_dist(self.poses[1][:4], self.poses[2][:4])
        #    )
        return self.rotation_distances
    
    def depth_num(self):
        
        num = self.depth_image[self.depth_image > 2]
        return num.size
        
    def z_tilt(self):
        tilts = []
        for pose in self.poses:
            tilt = 1 - np.abs(1-2*(pose[1]**2 + pose[2]**2))
            tilts.append(tilt)

        return tilts
    
    def estimatePose(self):
        img1 = self.images[0]
        img2 = self.images[1]
        
        sift = cv2.SIFT_create()
        kp1, des1 = sift.detectAndCompute(img1,None)

        kp2, des2 = sift.detectAndCompute(img2,None)



        bf = cv2.BFMatcher_create()
        matches = bf.match(des1, des2)

        src_points = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_points = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        K = np.array([[224.05, 0, 188.25],
                      [0, 224.05, 120.25],
                      [0, 0, 1]   ])

        E, mask = cv2.findEssentialMat(src_points, dst_points, K)
        _, R, t, mask = cv2.recoverPose(E, src_points, dst_points, K, mask)
        
        #t *= np.linalg.norm(self.poses[0][4:] - self.poses[1][4:])
        quat = q.Quaternion(matrix=R)
        self.t = t
        self.quat = quat
        self.R = R

        return quat, t
    
    
    def estimationDiff(self):
        quat = self.quat

        est_dist = quat.absolute_distance(quat, q.Quaternion())
        
        q0 = q.Quaternion(self.poses[0][:4])
        q1 = q.Quaternion(self.poses[1][:4])
        gt_dist = quat.absolute_distance(q0, q1)

        print(est_dist, gt_dist)
        

        return est_dist, gt_dist
    
    def translationDiff(self):
        t0 = self.poses[0][4:]

        t1 = self.poses[1][4:]

        print(self.poses[1][4:])
        print(t1)

        diff = t1 - t0
        print(diff)
        leng = np.linalg.norm(diff)
        print(leng)
        direction = diff/leng
        
            
        direction_diff = self.t.flatten() - direction
        return direction_diff, leng

        

class AnalysisWindow:

    def __init__(self):

        self.rgb_images = []
        self.depth_images = []

        self.window = gui.Application.instance.create_window(
            "Analyse", 1000, 500)
        self.window.set_on_layout(self._on_layout)
        self.window.set_on_close(self._on_close)

        self.widget3d = gui.SceneWidget()
        self.widget3d.scene = rendering.Open3DScene(self.window.renderer)
        self.window.add_child(self.widget3d)

        em = self.window.theme.font_size
        margin = 0.5 * em
        self.panel = gui.Vert(0.5 * em, gui.Margins(margin))
        self.panel.add_child(gui.Label("Color image"))
        self.rgb_widget = gui.ImageWidget(self.rgb_images[0])
        self.panel.add_child(self.rgb_widget)
        self.panel.add_child(gui.Label("Depth image (normalized)"))
        self.depth_widget = gui.ImageWidget(self.depth_images[0])
        self.panel.add_child(self.depth_widget)
        self.window.add_child(self.panel)

        self.is_done = False
        threading.Thread(target=self._update_thread).start()

    def _on_layout(self, layout_context):
        contentRect = self.window.content_rect
        panel_width = 15 * layout_context.theme.font_size  # 15 ems wide
        self.widget3d.frame = gui.Rect(contentRect.x, contentRect.y,
                                       contentRect.width - panel_width,
                                       contentRect.height)
        self.panel.frame = gui.Rect(self.widget3d.frame.get_right(),
                                    contentRect.y, panel_width,
                                    contentRect.height)

    def _on_close(self):
        self.is_done = True
        return True  # False would cancel the close
    
    

    def _update_thread(self):
        # This is NOT the UI thread, need to call post_to_main_thread() to update
        # the scene or any part of the UI.
        idx = 0
        while not self.is_done:
            time.sleep(0.100)

            # Get the next frame, for instance, reading a frame from the camera.
            rgb_frame = self.rgb_images[idx]
            depth_frame = self.depth_images[idx]
            idx += 1
            if idx >= len(self.rgb_images):
                idx = 0

            # Update the images. This must be done on the UI thread.
            def update():
                self.rgb_widget.update_image(rgb_frame)
                self.depth_widget.update_image(depth_frame)
                self.widget3d.scene.set_background([1, 1, 1, 1], rgb_frame)

            if not self.is_done:
                gui.Application.instance.post_to_main_thread(
                    self.window, update)
    
counter = 0
def cb_right(vis):
    vis.clear_geometries()
    for i in range(globals()["counter"], globals()["counter"]+2):
        globals()["counter"] += 1
        pose = poses[i]
        local_i = i % 2
        quat = pose[:4]
        quat_x_first = np.zeros(4)
        quat_x_first[:3] = quat[1:]
        quat_x_first[3] = quat[0]
        t = pose[4:]
        identity = o3d.geometry.TriangleMesh.create_coordinate_frame()
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
        rot = mesh.get_rotation_matrix_from_quaternion(quat)
        mesh.rotate(rot)
        mesh.translate(t)

        if i == 0:
            reset_view = True
        else:
            reset_view = False


        vis.add_geometry(identity, reset_view)
        vis.add_geometry(mesh, reset_view)

        if local_i == 2:
            
            
            return False




        
episodes = []
for i in range(0, len(poses), 2):
    episode = PlaneSweepEpisode(poses[i:i+2], images[i:i+2], depth_images[i//2])
    episodes.append(episode)

dists = []
depth_nums = []
tilts = []
est_dist_diff = []
x_diff = []
y_diff = []
z_diff = []
t_error = []
sum_error = []
pose_length = []
for episode in episodes:
    dist = max(episode.compute_roatation_distances())
    num = episode.depth_num()
    tilt = episode.z_tilt()
    episode.estimatePose()
    est_dist, gt_dist = episode.estimationDiff()
    dir_diff, leng = episode.translationDiff()
    rot_error = np.abs(est_dist - gt_dist)
    t_err = np.linalg.norm(dir_diff)
   
    dists.append(dist)
    depth_nums.append(num)
    tilts.append(sum(tilt))
    
    est_dist_diff.append(rot_error)
    x_diff.append(dir_diff[0])
    y_diff.append(dir_diff[1])
    z_diff.append(dir_diff[2])
    t_error.append(t_err)
    sum_error.append(rot_error + t_err)
    pose_length.append(leng)


plt.plot(dists, ".", label="relative rotation")
plt.legend()
plt.figure()
plt.plot(tilts, ".", label="tilt")
plt.legend()
plt.figure()
plt.plot(depth_nums, ".", label="depth(n)")
plt.legend()
plt.figure()
plt.plot(est_dist_diff, ".", label="rotation error")
plt.legend()
plt.figure()
plt.plot(t_error, ".", label="t_error")
plt.legend()
plt.figure()
plt.plot(sum_error, ".", label="sum_error")
plt.legend()
plt.figure()
plt.plot(pose_length, ".", label="travel length")
plt.legend()
"""plt.figure()
plt.plot(x_diff, ".", label="x")
plt.legend()
plt.figure()
plt.plot(y_diff, ".", label="y")
plt.legend()
plt.figure()
plt.plot(z_diff, ".", label="z")
plt.legend()"""
plt.show()

"""key_callback = {}
key_callback[ord("D")] = cb_right

mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()

o3d.visualization.draw_geometries_with_key_callbacks([], key_callback)"""