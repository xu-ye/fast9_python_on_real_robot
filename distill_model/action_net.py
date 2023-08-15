import torch
import torch.nn as nn
from torch.nn import functional as F
import math
import numpy as np


def init_(m, gain=0.01, activate=False):
    if activate:
        gain = nn.init.calculate_gain('relu')
    return init(m, nn.init.orthogonal_, lambda x: nn.init.constant_(x, 0), gain=gain)


def init(module, weight_init, bias_init, gain=1):
    weight_init(module.weight.data, gain=gain)
    if module.bias is not None:
        bias_init(module.bias.data)
    return module
class ActionNet(nn.Module):
    """ an unassuming Transformer block """

    def __init__(self, obs_dim, n_embd, action_dim,device):
        super(ActionNet, self).__init__()

        self.ln1 = nn.LayerNorm(obs_dim)
       
        self.mlp = nn.Sequential(
            init_(nn.Linear(obs_dim, 1 * n_embd), activate=True),
            nn.GELU(),
            init_(nn.Linear( n_embd, n_embd)),
            nn.GELU(),
            init_(nn.Linear( n_embd, n_embd)),
            nn.GELU(),
            init_(nn.Linear( n_embd, n_embd)),
            nn.GELU(),
            init_(nn.Linear( n_embd, n_embd)),
            nn.GELU(),
            init_(nn.Linear( n_embd, int(n_embd/4))),
            nn.GELU(),
            init_(nn.Linear( int(n_embd/4), action_dim)),
        )
        self.mlp2 = nn.Sequential(
            nn.Linear(obs_dim, 1 * n_embd),
            nn.GELU(),
            nn.Linear( n_embd, n_embd),
            nn.GELU(),
            nn.Linear( n_embd, n_embd),
            
        )
        self.mlp3 = nn.Sequential(
           
            nn.Linear( n_embd, n_embd),
            nn.GELU(),
            nn.Linear( n_embd, n_embd),
            nn.GELU(),
            nn.Linear( n_embd, n_embd),
            nn.GELU(),
            nn.Linear( n_embd, int(n_embd/4)),
            nn.GELU(),
            nn.Linear( int(n_embd/4), action_dim),
        )
        self.to(device)
        

    def forward(self, x):
        x1=x.to(torch.float32)
        #x = self.ln1(x1)   
        y = self.mlp2(x1)
        y = self.mlp3(y)
        return y

