# DNN Visual Global Localization for Autonomous Robots

A deep neural network approach for global visual localization, running in real time on low-cost GPUs.


## Related Works:

- **SABGL:** https://github.com/LCAD-UFES/SABGL
- **WNN-CNN-GL:** https://github.com/LCAD-UFES/WNN-CNN-GL

## Get started

Download all image folders and respective camera pose files

- UFES's dataset download link: https://drive.google.com/drive/u/1/folders/1tqRKGO3DtW1yreoxYeD9Ssc3Ip8fxaXC

Set the following parameters on \'script/config.txt\':
> - image_path="/dados/ufes/" `(path to images from link above)`
> - output_path="/dados/ufes_gt/" `(path to output directory)`
> - base_offset=5 `(spacing between key frames)`
> - live_offset=1 `(spacing between live frames)`

Run the following code:

>./scripts/dataset.sh

needs to have python 2.7 installed on system.

