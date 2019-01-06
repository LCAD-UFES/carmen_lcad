
import numpy as np
import torch

lines = [l.rstrip().rsplit() for l in open('/dados/data/data_log-estacionamento-ambiental-20181208.txt/sync.txt', 'r').readlines()]

poses = [torch.randn(3, requires_grad=True) for _ in range(len(lines))]
optim = torch.optim.LBFGS(poses, lr=1e-2)
x = torch.ones(2, 2, requires_grad=True)

def closure():
    optim.zero_grad()
    loss = 0.
    for i in range(len(lines)):
        x = float(lines[i][13]) - float(lines[0][13])
        y = float(lines[i][14]) - float(lines[0][14])
        loss += ((poses[i][0] - x) ** 2 + (poses[i][1] - y) ** 2) ** 0.5
        if i > 0:
            dt = float(lines[i][16]) - float(lines[i-1][16])
            ds = dt * float(lines[i-1][23])
            px = poses[i-1][0] + ds * torch.cos(poses[i-1][2])
            py = poses[i-1][1] + ds * torch.sin(poses[i-1][2])
            pth = poses[i-1][2] + ds * np.tan(float(lines[i-1][24])) * (1. / 2.625)
            loss += ((poses[i][0] - px) ** 2 + (poses[i][1] - py) ** 2) ** 0.5 
            loss += ((torch.sin(poses[i][2]) - torch.sin(pth)) ** 2 + (torch.cos(poses[i][2]) - torch.cos(pth)) ** 2) ** 0.5
    loss.backward()
    return loss

for epoch in range(100):
    optim.step(closure)
    print(closure(), poses[0], poses[-1])

