# YOLOv5 tutorial
This guide explains how to set up, train, convert and deploy a YOLOv5 model for this project.

## Directory structure
After training, your directory structure should be organized as follows:

```
datasets
└── polygons
    ├── test
    │   ├── images
    │   └── labels
    ├── train
    │   ├── images
    │   └── labels
    └── val
        ├── images
        └── labels
yolov5
├── data
│   └── polygons.yaml
├── runs
│   └── train
│       └── exp
│           └── weights
│               └── best.pt
└── train.py
```

## Dataset
Download the `polygons` dataset from [this repository](https://github.com/Patrycj2a/praca_inzynierska/tree/main/datasets/polygons/) and place it in the `datasets` folder.

## Training configuration file
Create the `polygons.yaml` file inside the `yolov5/data` directory with the following content:

```yaml
names:
  0: tetragon
  1: hexagon
  2: octagon
  3: dodecagon
nc: 4
path: ../datasets/polygons
train: train/images
val: val/images
test: test/images
```

## YOLOv5 setup
Set up the YOLOv5 repository by running the following commands:

```bash
git clone https://github.com/ultralytics/yolov5.git
cd yolov5
git pull
pip install -U -r yolov5/requirements.txt
```

## Model training
Train the YOLOv5 model using the following command:

```bash
python3 train.py --img 416 --batch 16 --epochs 1000 --data polygons.yaml --weights yolov5s.pt --cos-lr
```

The trained model weights will be saved as `best.pt` in the `yolov5/runs/train/exp/weights` directory.

## Model conversion
1. Go to [Luxonis Tools](https://tools.luxonis.com/).
2. Set `Yolo Version` to `YOLOv5`.
3. Click `File` and upload the obtained model weights (`best.pt`).
4. Set `Input image shape` to `416`.
5. In advanced options, set `Shaves` to `5`.
6. Click `Submit` to start the conversion.
7. Extract the downloaded ZIP file and locate the model named `best_openvino_2022.1_5shave.blob`.

## Model placement
Move the `best_openvino_2022.1_5shave.blob` file to the `ceai_models` in your home directory and rename it to `yolo.blob`.
