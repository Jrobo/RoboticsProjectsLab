import pyrealsense2 as rs
import numpy as np
import cv2

def run_depth_segmentation():
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)

    # Get depth scale for converting to meters
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    align = rs.align(rs.stream.color)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Convert depth to meters (float)
            depth_m = depth_image.astype(float) * depth_scale

            # Find nearest non-zero depth value (the closest object)
            non_zero = depth_m[depth_m > 0]
            if non_zero.size == 0:
                cv2.putText(color_image, "No depth", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)
                cv2.imshow("RGB Segmentation", color_image)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                continue

            min_depth = np.min(non_zero)

            # Create a mask for close objects near the minimum depth
            # Tweak delta (meters) to include object size; user can adjust with trackbar
            delta = 0.08
            mask_near = np.logical_and(depth_m > 0, depth_m <= (min_depth + delta)).astype(np.uint8) * 255

            # Clean up mask
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
            mask_clean = cv2.morphologyEx(mask_near, cv2.MORPH_OPEN, kernel, iterations=2)
            mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel, iterations=2)

            # Find contours and pick the largest as the object
            contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            seg_mask = np.zeros_like(mask_clean)

            if contours:
                contours = sorted(contours, key=cv2.contourArea, reverse=True)
                # require reasonable area
                if cv2.contourArea(contours[0]) > 500:
                    cnt = contours[0]
                    x,y,w,h = cv2.boundingRect(cnt)

                    # Initialize GrabCut with rectangle from depth-based bbox
                    gc_mask = np.zeros(color_image.shape[:2], np.uint8)
                    rect = (x, y, w, h)
                    try:
                        bgdModel = np.zeros((1,65), np.float64)
                        fgdModel = np.zeros((1,65), np.float64)
                        cv2.grabCut(color_image, gc_mask, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)
                        grabcut_mask = np.where((gc_mask==2)|(gc_mask==0), 0, 1).astype('uint8')
                        seg_mask = (grabcut_mask * 255).astype(np.uint8)
                    except Exception:
                        # fallback to depth mask
                        seg_mask = mask_clean

                    # compute depth stats within segmentation
                    seg_indices = seg_mask.astype(bool)
                    if np.count_nonzero(seg_indices):
                        depths_in_mask = depth_m[seg_indices]
                        mean_m = float(np.mean(depths_in_mask))
                        median_m = float(np.median(depths_in_mask))
                        mean_mm = mean_m * 1000.0
                        median_mm = median_m * 1000.0

                        # Draw results
                        cv2.rectangle(color_image, (x,y), (x+w, y+h), (0,255,0), 2)
                        cv2.drawContours(color_image, [cnt], -1, (0,128,255), 2)
                        text = f"Mean: {mean_mm:.0f} mm / {mean_mm/10:.1f} cm"
                        cv2.putText(color_image, text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2)
                        text2 = f"Median: {median_mm:.0f} mm"
                        cv2.putText(color_image, text2, (x, y-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),1)
                    else:
                        cv2.putText(color_image, "No segmentation depth", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
                else:
                    cv2.putText(color_image, "No object (small) detected", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
            else:
                cv2.putText(color_image, "No object detected", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)

            # Overlay segmentation mask (if available)
            if seg_mask is not None and seg_mask.any():
                colored_mask = cv2.applyColorMap(cv2.convertScaleAbs(seg_mask, alpha=1.0), cv2.COLORMAP_JET)
                overlay = cv2.addWeighted(color_image, 0.7, colored_mask, 0.3, 0)
            else:
                overlay = color_image

            # Also show a depth colormap for reference
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            combined = np.hstack((overlay, depth_colormap))

            cv2.imshow("RGB Segmentation", combined)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    run_depth_segmentation()
