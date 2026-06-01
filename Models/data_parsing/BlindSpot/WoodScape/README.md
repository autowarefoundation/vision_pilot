# Blind‑Spot Occupancy Annotation Tool

## 1. Purpose

This repository provides a manual annotation tool for labeling **blind‑spot occupancy** using **fisheye camera images** from the WoodScape dataset.

Each image is labeled as:

*   **OCCUPIED** – blind spot contains a relevant object
*   **FREE** – blind spot is clear

The tool supports left/right fisheye cameras, resume‑safe annotation, and visual QA overlays.

***

## 2. Dataset Download

The WoodScape dataset is provided by Valeo and can be downloaded from:

<https://woodscape.valeo.com/woodscape/>

Please follow the dataset license and usage terms provided on the official website.  
This repository does **not** redistribute the dataset.

***

## 3. Dataset Preparation

### 3.1 Original WoodScape folder structure

After downloading and extracting WoodScape, the relevant folders are:

*   rgb_images/rgb_images

*   rgb_images/rgb_images(test_set)

*   previous_images/previous_images

*   previous_images/previous_images(test_set)

Both **rgb_images** and **previous_images** contain fisheye images from multiple cameras, including **left and right fisheye**.

***

### 3.2 Arrange images for annotation

You must collect **all left and right fisheye images** from **all of the following folders**:

*   rgb_images/rgb_images
*   rgb_images/rgb_images(test_set)
*   previous_images/previous_images
*   previous_images/previous_images(test_set)

and place them into a unified structure for annotation.

### Required structure


```text
all_images/
├── left_fisheye/
│   └── images/
│       └── *.png
├── right_fisheye/
│   └── images/
│       └── *.png
```


### What to do

1.  From rgb_images/rgb_images and rgb_images/rgb_images(test_set):
    *   Copy **all left fisheye images** into left_fisheye/images
    *   Copy **all right fisheye images** into right_fisheye/images

2.  From previous_images/previous_images and previous_images/previous_images(test_set):
    *   Copy **all left fisheye images** into left_fisheye/images
    *   Copy **all right fisheye images** into right_fisheye/images

Images from rgb_images and previous_images are intentionally **merged** in the destination folders.  
Do **not** mix left and right cameras.

***

## 4. Environment Variable Setup

The annotation script locates the dataset via an environment variable.

Set the variable:

```bash
export WOODSCAPE_DATA_DIR=/absolute/path/to/all_images
```

Example:

```bash
export WOODSCAPE_DATA_DIR=/home/bilyas1/WoodScape/data/all_images
```

Verify:

```bash
echo $WOODSCAPE_DATA_DIR
ls $WOODSCAPE_DATA_DIR
```

You should see `left_fisheye` and `right_fisheye`.

The script will fail early with a clear error if the variable is missing or incorrect.

***

## 5. Installation Requirements

### Python

Python 3.8 or newer

### Dependencies

```bash
pip install opencv-python numpy
```

(Optional)

```bash
pip install -r requirements.txt
```

***

## 6. Running the Annotation Tool

Start the tool:

```bash
python annotate_data.py
```

You will be prompted to select the camera:

1 → left\_fisheye  
2 → right\_fisheye

***

## 7. Annotation Controls

While the OpenCV window is open:

*   **t** → OCCUPIED (True)
*   **f** → FREE (False)
*   **q** → Quit safely (progress saved)

After each decision, a short color overlay is shown:

*   Red → OCCUPIED
*   Green → FREE

***

## 8. Output

For each camera folder, the tool generates:

*   annotation_visualizations/  
    (images with red/green overlays)
*   occupancy_annotations.json

Example format of occupancy_annotations.json:

```
{
  "000123": true,
  "000124": false
}
```

Annotations are written **after every image**, so the tool can be stopped and resumed safely.

***

## 9. Resume Behavior

*   Already annotated images are skipped automatically
*   You can quit at any time using `q`
*   Restarting the script continues where you left off

***
