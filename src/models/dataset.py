import sys
import os
import numpy as np
from pandas import read_csv
from skimage import io, transform
from torch.utils.data import Dataset

value_range = [
    (-0.5,  0.5), # angle        
    (-7,   -2.5), # toMarking_L  
    (-2,    3.5), # toMarking_M  
    ( 2.5,  7),   # toMarking_R  
    ( 0,    75),  # dist_L       
    ( 0,    75),  # dist_R       
    (-9.5, -4),   # toMarking_LL 
    (-5.5, -0.5), # toMarking_ML 
    ( 0.5,  5.5), # toMarking_MR 
    ( 4,    9.5), # toMarking_RR 
    ( 0,    75),  # dist_LL      
    ( 0,    75),  # dist_MM      
    ( 0,    75),  # dist_RR      
    ( 0,    1)    # fast         
]

min_nv = 0.1
max_nv = 0.9

def normalize(av):
    def f(v, r):
        v = float(v)
        min_v = float(r[0])
        max_v = float(r[1])
        v = (v - min_v) / (max_v - min_v)
        v = v * (max_nv - min_nv) + min_nv
        v = min(max(v, 0.0), 1.0)
        return v

    for (i, v) in enumerate(av):
        av[i] = f(v, value_range[i])
    
    return av

def denormalize(av):
    def f(v, r):
        v = float(v)
        min_v = float(r[0])
        max_v = float(r[1])
        v = (v - min_nv) / (max_nv - min_nv)
        v = v * (max_v - min_v) + min_v
        return v

    for (i, v) in enumerate(av):
        av[i] = f(v, value_range[i])
    
    return av

class CarlaDataset(Dataset):
    """CARLA dataset."""

    def __init__(self, csv_file, root_dir, valid, transform=None):
        self.metadata = read_csv(csv_file, header=None)
        self.root_dir = root_dir
        self.transform = transform
        self.valid = valid

    def __len__(self):
        return len(self.metadata)

    def __getitem__(self, idx):
        img_id = self.metadata.iloc[idx,0].split('-')
        img_name = os.path.join(self.root_dir, img_id[0], img_id[1], "{}.png".format(img_id[2]))
        image = io.imread(img_name)
               
        # Delete alpha channel
        if image.shape[-1] == 4:
            image = np.delete(image, 3, 2)
        
        # Scale to 280x210
        image = transform.resize(image, (210, 280, 3), mode='constant', anti_aliasing=True)

        # Make it CHW
        image = image.transpose(2, 0, 1).astype('float32')
               
        av = self.metadata.iloc[idx,1:].values
        av = av.astype('float32')
        av = av[self.valid]
        av = normalize(av)
        sample = {'image': image, 'affordance_vector': av}

        if self.transform:
            sample = self.transform(sample)

        return sample

