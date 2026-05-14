## AutoDrive 2.0 - temporal end-to-end distance and curvature estimation

For autonomous cruise and driver-assistance applications, it is important to estimate both the road-following demand and the lead-object distance from camera input. AutoDrive is a compact temporal model that predicts three outputs from consecutive front-view frames: **distance-to-CIPO**, **road curvature**, and **CIPO presence probability**.

AutoDrive processes a pair of frames (`t-1`, `t`) and uses a shared-feature temporal fusion head to capture short-term motion cues that a single-frame model can miss. It is designed for 2:1 input aspect ratio and is typically used with the same center-crop preprocessing used during training.

### Demo Video

[![Watch the Video](../../../Media/AutoDrive_thumbnail.jpg)](https://drive.google.com/file/d/1Lh7uIslZMond8p-vavX6jxtgzU1goETb/preview)
## Get Started

To quickly try AutoDrive on your own data, please follow the steps in the [tutorial](tutorial.ipynb).  
For best results, ensure your inference input follows the training preprocessing pipeline (50-degree center crop and 1024x512 resize).

### Performance Results

AutoDrive is trained as a multi-task regression/classification model on sequence data with labels for curvature, CIPO distance, and CIPO presence.  

- Curvature Accuracy: average error of **4 degrees** of steering wheel angle
- CIPO Presence: **88% accuracy**
- Distance Accuracy in range bins: 

    5m - 40m (**90%** CIPO distance measurement accuracy) 

    40m - 70m (**91%** CIPO ditance measurement accuracy)

    80m - 100m (**86%** CIPO distance mesaurement accuracy)

    100m - 150m (**72%** CIPO distance measurement accuracy)

## Model variants

**AutoDrive model weights:**

### [Link to Download Pytorch Model Weights *.pth](https://drive.google.com/file/d/1FyyvmGvh4C96nKivsNCUAwf4cnorl_3V/view?usp=drive_link)
### [Link to Download ONNX FP32 Weights *.onnx](https://drive.google.com/file/d/1GKqhrNP5xtLBRqrcqL9k1IKnwicSL-N8/view?usp=drive_link)
### [Link to Download ONNX INT8 Weights *.onnx](https://drive.google.com/file/d/1n0NqVnyf2Ry6wlaORyC8-xtTIZhdEhcT/view?usp=drive_link)

### Notes

- Training entry point: `Models/training/train_auto_drive.py`
- Core network: `Models/model_components/autodrive/autodrive_network.py`
- Data preprocessing and scaling: `Models/data_utils/load_data_auto_drive.py`
- AutoSpeed backbone warm-start is supported through `--autospeed-ckpt`
