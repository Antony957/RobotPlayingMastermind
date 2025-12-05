import cv2
import numpy as np

color_ranges = {
    "yellow": ([15, 80, 60], [40, 255, 255]),
    "blue": ([95, 80, 40], [125, 255, 255]),
    "purple": ([120, 20, 40], [150, 255, 255]),
    # red/pink sits near hue 10; expand into red range
    "red": ([0, 45, 35], [20, 250, 250]),
    "red2": ([160, 45, 35], [190, 255, 255]),
    # green around 50
    "green": ([40, 40, 40], [70, 255, 255]),
    "black": ([0, 0, 0], [180, 80, 70]),
}

blur_amt = 17

# color_ranges = {
#     "yellow": ([0, 100, 100], [10, 255, 255]),
#     "red": ([170, 100, 100], [180, 255, 255]),
#     "blue": ([100, 100, 100], [140, 255, 255]),
#     "green": ([20, 100, 100], [35, 255, 255]),
#     "purple": ([140, 100, 100], [160, 255, 255]),
#     "black": ([0, 0, 0], [180, 255, 40]),
# }


def segment(image):
    # try:
    #     ok = cv2.imwrite("./debug.png", image)
    #     if not ok:
    #         print("Failed to write debug image to ./debug.png")
    # except Exception as e:
    #     print(f"Exception while saving debug image: {e}")

    height, width, _ = image.shape

    # --- 1. ROI Cropping (Updated for closer camera) ---
    roi_y_start = int(height * 0.5)
    roi_y_end = int(height * 0.7)
    roi_x_start = int(width * 0.15)
    roi_x_end = int(width * 0.95)
    roi_img = image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
    cv2.imwrite("./images/debug_crop.png", roi_img)

    hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
    if blur_amt:
        hsv = cv2.GaussianBlur(hsv, (blur_amt, blur_amt), 0)
    detected_blocks = []

    # --- 2. Color Detection ---
    for color_name, (lower, upper) in color_ranges.items():
        # if color_name == 'red':
        #     l2, u2 = self.color_ranges['red2']
        #     mask = cv2.inRange(hsv, np.array(lower), np.array(upper)) + \
        #            cv2.inRange(hsv, np.array(l2), np.array(u2))
        # elif color_name == 'red2':
        #     continue
        # else:
        #     mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

        if color_name == "red2":
            color_name = "red"

        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imwrite(f"./images/debug_mask_{color_name}.png", mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_contour = max(contours, key=cv2.contourArea)
        largest_area = cv2.contourArea(largest_contour)

        if largest_area >= 4000:
            M = cv2.moments(largest_contour)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])

            # Append centroid x position for sorting
            detected_blocks.append({"color": color_name, "cx": cx})

    # Sort left to right
    detected_blocks.sort(key=lambda b: b["cx"])

    # get colors only
    detected_colors = [k["color"] for k in detected_blocks]

    return detected_colors


if __name__ == "__main__":
    import os

    from PIL import Image

    img_dir = "./images"
    images = [
        e
        for e in os.scandir(img_dir)
        if e.is_file()
        and e.name.split(".")[-1] in ["jpg", "jpeg", "png"]
        and "debug" not in e.name
    ]
    images.sort(key=lambda e: e.name)

    for el in images:
        print(f"Processing image {el.name}")
        pil_img = Image.open(el)
        cv_img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
        detected_blocks = segment(cv_img)
        print(f"image {el.name} has {detected_blocks}")
