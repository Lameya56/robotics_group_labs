import random
from collections import deque
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F


def _to_tensor(x, dtype=torch.float):
    return torch.tensor(x, dtype=dtype)

# Q-Network Model
class LinearQNet(nn.Module):
    def __init__(self, input_size: int, hidden_size: int, output_size: int):
        super().__init__()
        self.linear1 = nn.Linear(input_size, hidden_size)
        self.linear2 = nn.Linear(hidden_size, hidden_size)
        self.linear3 = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        x = F.relu(self.linear1(x))
        x = F.relu(self.linear2(x))
        return self.linear3(x)

    def save(self, path: str):
        torch.save(self.state_dict(), path)

    def load(self, path: str, device: torch.device):
        self.load_state_dict(torch.load(path, map_location=device))

# Q-Learning Trainer
class QTrainer:
    def __init__(self, model: nn.Module, lr: float, gamma: float, device: torch.device):
        self.lr = lr
        self.gamma = gamma
        self.model = model
        self.device = device
        self.optimizer = optim.Adam(model.parameters(), lr=self.lr)
        self.criterion = nn.MSELoss()

    def train_step(self, state, action, reward, next_state, done):
        state = _to_tensor(state).to(self.device)
        next_state = _to_tensor(next_state).to(self.device)
        action = _to_tensor(action, dtype=torch.long).to(self.device)
        reward = _to_tensor(reward).to(self.device)

        if len(state.shape) == 1:
            state = torch.unsqueeze(state, 0)
            next_state = torch.unsqueeze(next_state, 0)
            action = torch.unsqueeze(action, 0)
            reward = torch.unsqueeze(reward, 0)
            done = (done,)

        pred = self.model(state)
        target = pred.clone().detach()

        for idx in range(len(done)):
            q_new = reward[idx]
            if not done[idx]:
                q_new = reward[idx] + self.gamma * torch.max(self.model(next_state[idx]))
            target[idx][action[idx].item()] = q_new

        self.optimizer.zero_grad()
        loss = self.criterion(pred, target)
        loss.backward()
        self.optimizer.step()


@dataclass
class AgentConfig:
    state_size: int
    action_size: int
    hidden_size: int = 256
    gamma: float = 0.99
    lr: float = 5e-4
    max_memory: int = 100_000
    batch_size: int = 512
    epsilon_start: float = 1.0
    epsilon_end: float = 0.01
    epsilon_decay: float = 0.998


class Agent:
    def __init__(self, config: AgentConfig):
        self.cfg = config
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.n_games = 0
        self.epsilon = config.epsilon_start
        self.memory = deque(maxlen=config.max_memory)
        self.model = LinearQNet(config.state_size, config.hidden_size, config.action_size).to(self.device)
        self.trainer = QTrainer(self.model, config.lr, config.gamma, self.device)

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def train_long_memory(self):
        if len(self.memory) > self.cfg.batch_size:
            mini_sample = random.sample(self.memory, self.cfg.batch_size)
        else:
            mini_sample = self.memory

        if not mini_sample:
            return

        states, actions, rewards, next_states, dones = zip(*mini_sample)
        self.trainer.train_step(states, actions, rewards, next_states, dones)

    def train_short_memory(self, state, action, reward, next_state, done):
        self.trainer.train_step(state, action, reward, next_state, done)

    def get_action(self, state: np.ndarray) -> int:
        self.epsilon = max(self.cfg.epsilon_end, self.epsilon * self.cfg.epsilon_decay)
        if random.random() < self.epsilon:
            return random.randint(0, self.cfg.action_size - 1)
        state0 = _to_tensor(state).to(self.device)
        preds = self.model(state0)
        return torch.argmax(preds).item()

    def save(self, path: str):
        self.model.save(path)

    def load(self, path: str):
        self.model.load(path, self.device)
