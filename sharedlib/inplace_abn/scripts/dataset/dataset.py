import glob
from itertools import chain
from os import path

import torch
from PIL import Image
from torch.utils.data import Dataset

import os
import numpy as np

class RDDFPredictDataset(Dataset):
    """RDDF Predict dataset."""

    def __init__(self, file, root_dir, transform=None):
        """
        Args:
            csv_file (string): Path to the csv file with annotations.
            root_dir (string): Directory with all the images.
            transform (callable, optional): Optional transform to be applied
                on a sample.
        """
        with open(file, 'r') as f:
            self.data = [line.strip().split(" ") for line in f]
        self.root_dir = root_dir
        self.transform = transform

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        img_name = os.path.join(self.root_dir,
                                self.data[idx][-1]+'-r.png')
        image = Image.open(img_name).convert(mode="RGB")

        if self.transform:
            image = self.transform(image)
            
        params = torch.from_numpy(np.array(self.data, np.float)[idx, :-1])
        sample = {'image': image, 'params': params}

        return sample

class SegmentationDataset(Dataset):
    _EXTENSIONS = ["*.jpg", "*.jpeg", "*.png"]

    def __init__(self, in_dir, transform):
        super(SegmentationDataset, self).__init__()

        self.in_dir = in_dir
        self.transform = transform

        # Find all images
        self.images = []
        for img_path in chain(*(glob.iglob(path.join(self.in_dir, ext)) for ext in SegmentationDataset._EXTENSIONS)):
            _, name_with_ext = path.split(img_path)
            idx, _ = path.splitext(name_with_ext)
            self.images.append({
                "idx": idx,
                "path": img_path
            })

    def __len__(self):
        return len(self.images)

    def __getitem__(self, item):
        # Load image
        with Image.open(self.images[item]["path"]) as img_raw:
            size = img_raw.size
            img = self.transform(img_raw.convert(mode="RGB"))

        return {"img": img, "meta": {"idx": self.images[item]["idx"], "size": size}}


def segmentation_collate(items):
    imgs = torch.stack([item["img"] for item in items])
    metas = [item["meta"] for item in items]

    return {"img": imgs, "meta": metas}
