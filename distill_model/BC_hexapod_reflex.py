#!/usr/bin/env python
import sys
import os
import wandb
import socket
import setproctitle
import numpy as np
from pathlib import Path
import torch
import gym
import gym_hexapod
#from stable_baselines.common.env_checker import check_env
import pybullet as p
import pybullet_data as pd
import math
import torch.nn as nn
import json
import Hexapod_Real
from action_net import *

sys.path.append("../../")
from mat.config import get_config
from mat.envs.ma_mujoco.multiagent_mujoco.mujoco_multi import MujocoMulti
from mat.runner.shared.mujoco_runner import MujocoRunner as Runner
from mat.envs.env_wrappers import ShareSubprocVecEnv, ShareDummyVecEnv
from mat.algorithms.mat.algorithm.transformer_policy import TransformerPolicy 
from mat.algorithms.mat.algorithm.ma_transformer import MultiAgentTransformer
from mat.utils.util import get_shape_from_obs_space, get_shape_from_act_space
from torch.utils.data import Dataset, DataLoader
#from tqdm import tqdm_notebook.tqdm as tqdm
from tqdm import tqdm


"""Train script for MuJoCo."""
def make_train_env(all_args):
    def get_env_fn(rank):
        def init_env():
            env = gym.make('hexapod_real-v0')
            '''
            if all_args.env_name == "mujoco":
                env_args = {"scenario": all_args.scenario,
                            "agent_conf": all_args.agent_conf,
                            "agent_obsk": all_args.agent_obsk,
                            "episode_limit": 1000}
                env = MujocoMulti(env_args=env_args)
            else:
                print("Can not support the " + all_args.env_name + "environment.")
                raise NotImplementedError
                '''
            #env.seed(all_args.seed + rank * 1000)
            return env

        return init_env

    if all_args.n_rollout_threads == 1:
        return ShareDummyVecEnv([get_env_fn(0)])
    else:
        return ShareSubprocVecEnv([get_env_fn(i) for i in range(all_args.n_rollout_threads)])


def make_eval_env(all_args):
    def get_env_fn(rank):
        def init_env():
            env = gym.make('hexapod_real-v0')
            return env

        return init_env

    if all_args.eval_episodes == 1:
        return ShareDummyVecEnv([get_env_fn(0)])
    else:
        return ShareSubprocVecEnv([get_env_fn(i) for i in range(all_args.eval_episodes)])


def parse_args(args, parser):
    parser.add_argument('--scenario', type=str, default='hexapod_real_reflex_bc-v0', help="Which mujoco task to run on")
    parser.add_argument('--agent_conf', type=str, default='4x2')
    parser.add_argument('--agent_obsk', type=int, default=0)
    parser.add_argument("--faulty_node",  type=int, default=-1)
    parser.add_argument("--eval_faulty_node", type=int, nargs='+', default=None)
    parser.add_argument("--add_move_state", action='store_true', default=False)
    parser.add_argument("--add_local_obs", action='store_true', default=False)
    parser.add_argument("--add_distance_state", action='store_true', default=False)
    parser.add_argument("--add_enemy_action_state", action='store_true', default=False)
    parser.add_argument("--add_agent_id", action='store_true', default=False)
    parser.add_argument("--add_visible_state", action='store_true', default=False)
    parser.add_argument("--add_xy_state", action='store_true', default=False)

    # agent-specific state should be designed carefully
    parser.add_argument("--use_state_agent", action='store_true', default=False)
    parser.add_argument("--use_mustalive", action='store_false', default=True)
    parser.add_argument("--add_center_xy", action='store_true', default=False)

    all_args = parser.parse_known_args(args)[0]

    return all_args

class NumpyArrayEncoder(json.JSONEncoder):
                    def default(self, obj):
                        if isinstance(obj, np.ndarray):
                            return obj.tolist()
                        return json.JSONEncoder.default(self, obj)



class Dataset_Expert(Dataset):
    def __init__(self, obs,actions) -> None:
        super().__init__()
        self.data = []
        for index in range(len(obs)):
            # obs, act = dat
            self.data.append([obs[index],actions[index]])
    
    def __len__(self):
        return len(self.data)
    
    def __getitem__(self, index):
        return self.data[index]
                       

def main(args):
    parser = get_config()
    all_args = parse_args(args, parser)
    all_args.n_history=20
    print("mumu config: ", all_args)

    if all_args.algorithm_name == "mat_dec":
        all_args.dec_actor = True
        all_args.share_actor = True

    # cuda
    if all_args.cuda and torch.cuda.is_available():
        print("choose to use gpu...")
        device = torch.device("cuda:1")
        torch.set_num_threads(all_args.n_training_threads)
        if all_args.cuda_deterministic:
            torch.backends.cudnn.benchmark = False
            torch.backends.cudnn.deterministic = True
    else:
        print("choose to use cpu...")
        device = torch.device("cpu")
        torch.set_num_threads(all_args.n_training_threads)

    run_dir = Path(os.path.split(os.path.dirname(os.path.abspath(__file__)))[
                       0] + "/results") / all_args.env_name / all_args.scenario / all_args.algorithm_name / all_args.experiment_name
    if not run_dir.exists():
        os.makedirs(str(run_dir))
    
    save_dir="/home/xuye/Downloads/mat2/mat/scripts/train/BC_model"

    if all_args.use_wandb:
        run = wandb.init(config=all_args,
                         project=all_args.env_name,
                         entity=all_args.user_name,
                         notes=socket.gethostname(),
                         name=str(all_args.algorithm_name) + "_" +
                              str(all_args.experiment_name) +
                              "_seed" + str(all_args.seed),
                         group=all_args.map_name,
                         dir=str(run_dir),
                         job_type="training",
                         reinit=True)
    else:
        if not run_dir.exists():
            curr_run = 'run1'
        else:
            exst_run_nums = [int(str(folder.name).split('run')[1]) for folder in run_dir.iterdir() if
                             str(folder.name).startswith('run')]
            if len(exst_run_nums) == 0:
                curr_run = 'run1'
            else:
                curr_run = 'run%i' % (max(exst_run_nums) + 1)
        run_dir = run_dir / curr_run
        if not run_dir.exists():
            os.makedirs(str(run_dir))

    setproctitle.setproctitle(
        str(all_args.algorithm_name) + "-" + str(all_args.env_name) + "-" + str(all_args.experiment_name) + "@" + str(
            all_args.user_name))

    # seed
    torch.manual_seed(all_args.seed)
    torch.cuda.manual_seed_all(all_args.seed)
    np.random.seed(all_args.seed)

    # env
    
      # set parameters 
    all_args.n_rollout_threads=16
    all_args.use_eval=False
    
    
    
    envs =  make_train_env(all_args)
    eval_envs = make_eval_env(all_args) if all_args.use_eval else None
    num_agents = envs.n_agents

    config = {
        "all_args": all_args,
        "envs": envs,
        "eval_envs": eval_envs,
        "num_agents": num_agents,
        "device": device,
        "run_dir": run_dir
    }
   
    #all_args.model_dir="/home/xuye/MAT/muti_hexapod/mat/scripts/results/mujoco/Ant-v5/mat/check/run59/models/transformer_1200.pt"
    all_args.model_dir="/home/xuye/MAT/muti_hexapod/mat/scripts/results/mujoco/HexapodReal-v0/mat/check/run9/models/transformer_200.pt"
    model_dir=all_args.model_dir
    all_args.n_history=1
    
    
        
    
    ## set model
    obs_dim_t = get_shape_from_obs_space(envs.observation_space)[1]
    share_obs_dim = get_shape_from_obs_space(envs.share_observation_space)[1]
    act_dim = envs.action_space.shape[1]
    action_type = 'Continuous'
            
    transformer = MultiAgentTransformer( share_obs_dim,  obs_dim_t,  act_dim, num_agents,
                               n_block=all_args.n_block, n_embd=all_args.n_embd, n_head=all_args.n_head,
                               encode_state=all_args.encode_state, device=device,
                               action_type=action_type, dec_actor=all_args.dec_actor,
                               share_actor=all_args.share_actor)
    
    
    
    
    loss_func2 = nn.MSELoss().to(device)
    loss_func3 = nn.MSELoss().to(device)
    loss_func4 = nn.MSELoss().to(device)
    loss_func5 = nn.MSELoss().to(device)
    loss_func6 = nn.MSELoss().to(device)
    
    loss_func1 = nn.MSELoss().to(device)
   
    
    ## load para
    saved_transformer_ = torch.load(model_dir)
    transformer.load_state_dict(saved_transformer_)
    
    
    
    ## set action net 
    n_embd=64
    obs_dim=obs_dim_t*num_agents
    
    
    action_net2=ActionNet(obs_dim, n_embd, act_dim,device)
    action_net3=ActionNet(obs_dim, n_embd, act_dim,device)
    action_net4=ActionNet(obs_dim, n_embd, act_dim,device)
    action_net5=ActionNet(obs_dim, n_embd, act_dim,device)
    action_net6=ActionNet(obs_dim, n_embd, act_dim,device)
    
    action_net1=ActionNet(obs_dim, n_embd*1, act_dim,device)
    all_args.lr=5e-4
    
    
    
    optimizer2 = torch.optim.Adam( action_net2.parameters(),
                                          lr= all_args.lr, eps= all_args.opti_eps,
                                          )
    optimizer3 = torch.optim.Adam( action_net3.parameters(),
                                          lr= all_args.lr, eps= all_args.opti_eps,
                                          )
    optimizer4 = torch.optim.Adam( action_net4.parameters(),
                                          lr= all_args.lr, eps= all_args.opti_eps,
                                          )
    optimizer5 = torch.optim.Adam( action_net5.parameters(),
                                          lr= all_args.lr, eps= all_args.opti_eps,
                                          )
    optimizer6 = torch.optim.Adam( action_net6.parameters(),
                                          lr= all_args.lr, eps= all_args.opti_eps,
                                          )
    
    optimizer1 = torch.optim.Adam( action_net1.parameters(),
                                          lr= all_args.lr, eps= all_args.opti_eps,
                                          )
    
    ## 监督学习
    EPOCHES=1000
    episode_length=1200
    batch=100
    
    for e in tqdm(range(EPOCHES)):
        obs, share_obs, _=envs.reset()
        obs_tensor=torch.tensor(obs).to(device).clone()
        obs_buf_episode=[]
        act_buf_episode=[]
        
        for step in range(episode_length):
            
            ls_ep = 0
            available_actions=None
            deterministic=True
            #torch.autograd.set_detect_anomaly(True)
            
            
            
            with torch.no_grad():
                obs_buf_episode.append(obs)
                actions_expert, action_log_probs, values = transformer.get_actions(share_obs,
                                                                                obs,
                                                                                available_actions,
                                                                                deterministic)
                action_np=actions_expert.cpu().detach().numpy()
                obs, obs_share, eval_rewards, eval_dones, eval_infos, _ =envs.step(action_np)
                act_buf_episode.append(action_np)
                #obs_tensor=torch.tensor(obs).to(device).clone()
                #ac1=actions_expert.detach()
        ld_demo = DataLoader(Dataset_Expert(obs_buf_episode,act_buf_episode), batch_size=batch,shuffle=True) 
        index_length=200
        for index_e in tqdm(range(index_length)):
            
            with tqdm(ld_demo) as TQ:
                ls_ep = 0
                ls_ep2 = 0
                ls_ep3 = 0
                ls_ep4 = 0
                ls_ep5 = 0
                ls_ep6 = 0
                index_count=0
            
                for obs, actions_expert in TQ:
                    index_count+=1
        
                    obs_tensor=torch.tensor(np.vstack(obs)).to(device).clone()
                    obs_a=torch.flatten(obs_tensor,1,2)
                    
                    action2=action_net2(obs_a)
                    action3=action_net3(obs_a)
                    action4=action_net4(obs_a)
                    action5=action_net5(obs_a)
                    action6=action_net6(obs_a)
                    
                    action1=action_net1(obs_a)
                    
                    
                    coef=100
                    action_tensor=torch.tensor(np.vstack(actions_expert)).to(device).clone()
                    
                    
                    ls_bh2 = loss_func2(action_tensor[:,1].to(device).float(),action2.to(device))*coef
                    ls_bh3 = loss_func3( action_tensor[:,2].to(device).float(),action3.to(device))*coef
                    ls_bh4 = loss_func4(action_tensor[:,3].to(device).float(),action4.to(device), )*coef
                    ls_bh5 = loss_func5(action_tensor[:,4].to(device).float(),action5.to(device))*coef
                    ls_bh6 = loss_func6(action_tensor[:,5].to(device).float(),action6.to(device), )*coef
                    ls_bh1 = loss_func1(action_tensor[:,0].to(device).float(),action1.to(device))*coef
                    #ls_bh1 = loss_func1(action1.to(device),action_tensor[:,0].to(device).float())*coef
                    
                    optimizer2.zero_grad()
                    ls_bh2.backward()
                    optimizer2.step()
                    
                    
                    
                    
                    
                    
                    optimizer3.zero_grad()
                    ls_bh3.backward()
                    optimizer3.step()
                    
                    optimizer4.zero_grad()
                    ls_bh4.backward()
                    optimizer4.step()
                    
                    optimizer5.zero_grad()
                    ls_bh5.backward()
                    optimizer5.step()
                    
                    optimizer6.zero_grad()
                    ls_bh6.backward()
                    optimizer6.step()
                    
                    
                    optimizer1.zero_grad()
                    ls_bh1.backward()
                    optimizer1.step()
                
                    ls_bh1 = ls_bh1.cpu().detach().numpy()
                    ls_bh2 = ls_bh2.cpu().detach().numpy()
                    ls_bh3 = ls_bh3.cpu().detach().numpy()
                    ls_bh4 = ls_bh4.cpu().detach().numpy()
                    ls_bh5 = ls_bh5.cpu().detach().numpy()
                    ls_bh6 = ls_bh6.cpu().detach().numpy()
            
                    ls_ep += ls_bh1
                    ls_ep2 += ls_bh2
                    ls_ep3 += ls_bh3
                    ls_ep4 += ls_bh4
                    ls_ep5 += ls_bh5
                    ls_ep6 += ls_bh6
                    
                    
                    
                    print("loss1",ls_bh1,"loss2",ls_bh2,"loss3",ls_bh3,"loss4",ls_bh4,"loss5",ls_bh5,"loss6",ls_bh6)
            
                ls_ep /= index_count
                ls_ep2 /= index_count
                ls_ep3 /= index_count
                ls_ep4 /= index_count
                ls_ep5 /= index_count
                ls_ep6 /= index_count
                print("\n")
                print('Ep %d: loss_policy=%.3f' % (e+1, ls_ep))
                print("index_e",index_e,"loss1",ls_ep,"loss2",ls_ep2,"loss3",ls_ep3,"loss4",ls_ep4,"ls_ep5",ls_bh5,"ls_ep6",ls_bh6)
            
        if e%10==0:
            print("save!")
            #torch.save(transformer.state_dict(), 'Model/model1_%d_st_%d.pt' % (all_args.n_history,e+1))
            torch.save(action_net1.state_dict(), str(save_dir) + "/action_net1_BC_mlp_" + str(all_args.n_history) + "_"+str(e)+".pt")
            torch.save(action_net2.state_dict(), str(save_dir) + "/action_net2_BC_mlp_" + str(all_args.n_history) + "_"+str(e)+".pt")
            torch.save(action_net3.state_dict(), str(save_dir) + "/action_net3_BC_mlp_" + str(all_args.n_history) + "_"+str(e)+".pt")
            torch.save(action_net4.state_dict(), str(save_dir) + "/action_net4_BC_mlp_" + str(all_args.n_history) + "_"+str(e)+".pt")
            torch.save(action_net5.state_dict(), str(save_dir) + "/action_net5_BC_mlp_" + str(all_args.n_history) + "_"+str(e)+".pt")
            torch.save(action_net6.state_dict(), str(save_dir) + "/action_net6_BC_mlp_" + str(all_args.n_history) + "_"+str(e)+".pt")
    

    

    # post process
 


if __name__ == "__main__":
    main(sys.argv[1:])
