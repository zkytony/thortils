# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

import numpy as np
import cv2
import shutil
import tarfile

__all__ = ['overlay',
           'cv2shape',
           'save_images_and_compress']

# CV2, image processing
def overlay(img1, img2, opacity=1.0, pos=(0,0)):
    """
    Returns an image that is the result of overlaying img2
    on top of img1. Works for both RGB and RGBA images.

    img1 is the background image. img2 is the image to
    put on top of img1. img1 will be modified.

    Note that for opencv images, x is the column, y is the row.
    """
    # Determine the pixels that will be affected
    x_offset, y_offset = pos
    assert 0 <= x_offset < img1.shape[1],\
        "Invalid x offset (%d). Acceptable range [0,%d)" % (x_offset, img1.shape[1])
    assert 0 <= y_offset < img1.shape[0],\
        "Invalid x offset (%d). Acceptable range [0,%d)" % (y_offset, img1.shape[0])
    # img1 region
    xs1, xf1 = x_offset, min(x_offset + img2.shape[1], img1.shape[1])
    ys1, yf1 = y_offset, min(y_offset + img2.shape[0], img1.shape[0])

    xs2, xf2 = 0, min(img2.shape[1], img1.shape[1] - x_offset)
    ys2, yf2 = 0, min(img2.shape[0], img1.shape[0] - y_offset)

    if img2.shape[2] == 4:
        # Get the alpha channel of img2
        alpha = opacity * (img2[ys2:yf2, xs2:xf2, 3] / 255.0)
    else:
        alpha = opacity

    for c in range(3):
        img1[ys1:yf1, xs1:xf1, c] =\
            (alpha * img2[ys2:yf2, xs2:xf2, c]\
             + (1.0-alpha) * img1[ys1:yf1, xs1:xf1, c])
    return img1

def cv2shape(img, func, *args, alpha=1.0, **kwargs):
    """Draws cv2 shape using `func` with arguments,
    on top of given image `img` that allows transparency."""
    img_paint = img.copy()
    func(img_paint, *args, **kwargs)
    img = cv2.addWeighted(img_paint, alpha, img, 1. - alpha, 0)
    return img


#### File Utils ####
def save_images_and_compress(images, outdir, filename="images", img_type="png"):
    # First write the images as temporary files into the outdir
    cur_time = dt.now()
    cur_time_str = cur_time.strftime("%Y%m%d%H%M%S%f")[:-3]
    img_save_dir = os.path.join(outdir, "tmp_imgs_%s" % cur_time_str)
    os.makedirs(img_save_dir)

    for i, img in enumerate(images):
        img = img.astype(np.float32)
        img = cv2.flip(img, 0)
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)  # rotate 90deg clockwise
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        save_path = os.path.join(img_save_dir, "tmp_img%d.%s" % (i, img_type))
        cv2.imwrite(save_path, img)

    # Then compress the image files in the outdir
    output_filepath = os.path.join(outdir, "%s.tar.gz" % filename)
    with tarfile.open(output_filepath, "w:gz") as tar:
        tar.add(img_save_dir, arcname=filename)

    # Then remove the temporary directory
    shutil.rmtree(img_save_dir)
