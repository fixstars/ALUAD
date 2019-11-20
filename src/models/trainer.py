import sys
import shutil
import os.path as path
import os
import torch
import torch.nn as nn
import torch.onnx  as onnx
import torch.optim as optim
from torch.utils.data import DataLoader
from torchsummary import summary
import dataset as ds
import deep_driving as dd
import tensorboardX as tbx
import argparse
import numpy as np
from torch.utils.data.sampler import SubsetRandomSampler
import logging
import glob

# torch.set_default_tensor_type('torch.cuda.FloatTensor')
parser = argparse.ArgumentParser(description='Training NN')
parser.add_argument('--root-dir','-rd',metavar='str', help='Root directory of the data',type=str,required=True)
parser.add_argument('--csv-name','-cn',metavar='str', help='CSV label file name',type=str,required=True)
parser.add_argument('--learning-rate','-lr',metavar='float', help='Learning rate',type=float,default=0.01)
parser.add_argument('--nn-name','-nn',metavar='str', help='Alex or Squeeze',type=str,default='alex')
args = vars(parser.parse_args())

if not os.path.exists('./log'):
    os.mkdir('./log') 
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
                    datefmt='%m-%d %H:%M',
                    filename='./log/log.txt',
                    filemode='w')
console = logging.StreamHandler()
console.setLevel(logging.INFO)
formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
console.setFormatter(formatter)
logging.getLogger('').addHandler(console)

valid = [0] + [i for i in range(6,14)]
net = dd.deep_driving(args['nn_name'],output=len(valid)).cuda()
criterion = nn.MSELoss(reduction='mean')
optimizer = optim.Adam(net.parameters(),lr=0.01)

train_writer = tbx.SummaryWriter('./log/train')
valid_writer = tbx.SummaryWriter('./log/valid')

# Enable to see model
summary(net, (3, 210, 280))

validation_split = .2
random_seed= 25
carla_dataset = ds.CarlaDataset(csv_file=path.join(args['root_dir'], args['csv_name']), 
                                root_dir=args['root_dir'], valid=valid)
dataset_size = len(carla_dataset)
indices = list(range(dataset_size))
split = int(np.floor(validation_split * dataset_size))
np.random.seed(random_seed)
np.random.shuffle(indices)
train_indices, val_indices = indices[split:], indices[:split]

# Creating PT data samplers and loaders:
train_sampler = SubsetRandomSampler(train_indices)
valid_sampler = SubsetRandomSampler(val_indices)

train_dataloader = DataLoader(carla_dataset, batch_size=64, num_workers=16, pin_memory=True, sampler=train_sampler)
validation_dataloader = DataLoader(carla_dataset, batch_size=64, num_workers=16, pin_memory=True, sampler=valid_sampler)

def init_normal(m):
    if type(m) == nn.Linear:
        nn.init.uniform_(m.weight)
net.apply(init_normal)

for epoch in range(20):
    train_loss_tot = 0
    train_num = 0 
    valid_loss_tot = 0
    valid_num = 0

    for i, train_sample in enumerate(train_dataloader):
        train_inputs = train_sample['image'].cuda()
        train_target = train_sample['affordance_vector'].cuda()

        train_outputs = net(train_inputs)
        #print(train_inputs.cpu())
        print('target:{}'.format(train_sample['affordance_vector'][0]))
        print('outputs:{}'.format(train_outputs.cpu()[0]))
        train_loss = criterion(train_outputs, train_target)

        optimizer.zero_grad()
        train_loss.backward()
        optimizer.step()
        logging.info("Epoch:{}, Sample:{}, Training Loss:{}".format(epoch,i,train_loss))
        train_loss_tot += train_loss
        train_num += train_inputs.size(0)
        #train_writer.add_scalar('Loss', train_loss.item(), epoch)

    with torch.no_grad():
        for i, valid_sample in enumerate(validation_dataloader):
            valid_inputs = valid_sample['image'].cuda()
            valid_target = valid_sample['affordance_vector'].cuda()
            valid_outputs = net(valid_inputs)
            valid_loss = criterion(valid_outputs,valid_target)
            logging.info("Epoch:{}, Sample:{}, Validating Loss:{}".format(epoch,i,valid_loss))
            valid_loss_tot += valid_loss
            valid_num += valid_inputs.size(0)
            #valid_writer.add_scalar('Loss', valid_loss.item(), epoch)

    logging.info("Epoch:{}, Training Samples:{}, Training Loss:{}".format(epoch,train_num,train_loss_tot/train_num))
    logging.info("Epoch:{}, Validating Samples:{}, Validating Loss:{}".format(epoch,valid_num,valid_loss_tot/valid_num))
    train_writer.add_scalar('Loss', train_loss_tot/train_num, epoch)
    valid_writer.add_scalar('Loss', valid_loss_tot/valid_num, epoch)

    #print('[%d/30, %5d/%5d] Training Loss: %.3f Validating Loss: %.3f' % (epoch, i, len(dataloader), train_loss.item(),valid_loss.item()))


torch.save(net.state_dict(), './dd_{}'.format(args['nn_name']))

train_writer.export_scalars_to_json('./train.json')
train_writer.close()
valid_writer.export_scalars_to_json('./valid.json')
valid_writer.close()

if not path.exists('./runs'):
    os.mkdir('./runs')
if not path.exists('./runs/train'): 
    shutil.move('./train','./runs')
    shutil.move('./valid','./runs')
else:
    for file in glob.glob(r'./train/*'):
        shutil.move(file,'./runs/train')
        os.rmdir('./train')
    for file in glob.glob(r'./valid/*'):
        shutil.move(file,'./runs/valid')
        os.rmdir('./valid')

onnx._export(net.cpu(), torch.randn(1, 3, 210, 280), "dd_{}.onnx".format(args['nn_name']), export_params=True)

logging.info(print('Finished Training'))
