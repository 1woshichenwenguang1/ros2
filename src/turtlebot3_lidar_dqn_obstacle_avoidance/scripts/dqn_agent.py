#!/usr/bin/env python3
"""Pure-numpy DQN building blocks for TurtleBot3 LiDAR obstacle avoidance.

No deep-learning framework is required: the Q network is a 3-layer MLP
(24-64-64-5) trained with experience replay, a target network and Adam.
"""
import json
import math
import random
from collections import deque

import numpy as np


def sectors_from_ranges(ranges, n_sectors, range_max=2.0):
    """Compress a LaserScan range list into n_sectors normalised distances."""
    out = [float(range_max)] * n_sectors
    m = len(ranges)
    if m == 0:
        return np.ones(n_sectors, dtype=np.float32)
    for i, v in enumerate(ranges):
        if v is None or math.isnan(v) or math.isinf(v) or v < 0:
            v = range_max
        v = min(v, range_max)
        sec = int(i * n_sectors / m)
        if sec >= n_sectors:
            sec = n_sectors - 1
        if v < out[sec]:
            out[sec] = v
    return (np.asarray(out, dtype=np.float32) / range_max).clip(0.0, 1.0)


class MLP(object):
    """Fully connected net with ReLU hides, linear output and Adam optimiser."""

    def __init__(self, sizes, seed=7):
        rng = np.random.RandomState(seed)
        self.sizes = list(sizes)
        self.W, self.b = [], []
        for i in range(len(sizes) - 1):
            self.W.append(rng.normal(0.0, math.sqrt(2.0 / sizes[i]),
                                     (sizes[i], sizes[i + 1])).astype(np.float64))
            self.b.append(np.zeros(sizes[i + 1]))
        self.mW = [np.zeros_like(w) for w in self.W]
        self.mb = [np.zeros_like(b) for b in self.b]
        self.vW = [np.zeros_like(w) for w in self.W]
        self.vb = [np.zeros_like(b) for b in self.b]
        self.t = 0

    def forward(self, x):
        a = np.atleast_2d(np.asarray(x, dtype=np.float64))
        cache = [a]
        for i in range(len(self.W) - 1):
            a = np.maximum(a.dot(self.W[i]) + self.b[i], 0.0)
            cache.append(a)
        out = a.dot(self.W[-1]) + self.b[-1]
        return out, cache

    def predict(self, x):
        out, _ = self.forward(x)
        return out

    def train(self, x, target, lr=1e-3):
        out, cache = self.forward(x)
        batch = out.shape[0]
        d = 2.0 * (out - target) / batch
        loss = float(np.mean((out - target) ** 2))
        grads = []
        for i in reversed(range(len(self.W))):
            a = cache[i]
            gW = a.T.dot(d)
            gb = d.sum(axis=0)
            grads.append((gW, gb))
            if i > 0:
                d = d.dot(self.W[i].T)
                d *= (cache[i] > 0).astype(np.float64)
        grads.reverse()
        self.t += 1
        b1, b2, eps = 0.9, 0.999, 1e-8
        for i, (gW, gb) in enumerate(grads):
            self.mW[i] = b1 * self.mW[i] + (1 - b1) * gW
            self.vW[i] = b2 * self.vW[i] + (1 - b2) * gW ** 2
            self.mb[i] = b1 * self.mb[i] + (1 - b1) * gb
            self.vb[i] = b2 * self.vb[i] + (1 - b2) * gb ** 2
            mW = self.mW[i] / (1 - b1 ** self.t)
            vW = self.vW[i] / (1 - b2 ** self.t)
            mb = self.mb[i] / (1 - b1 ** self.t)
            vb = self.vb[i] / (1 - b2 ** self.t)
            self.W[i] -= lr * mW / (np.sqrt(vW) + eps)
            self.b[i] -= lr * mb / (np.sqrt(vb) + eps)
        return loss

    def params(self):
        return {'sizes': self.sizes,
                'W': [w.tolist() for w in self.W],
                'b': [b.tolist() for b in self.b]}

    def load(self, data):
        self.W = [np.asarray(w, dtype=np.float64) for w in data['W']]
        self.b = [np.asarray(b, dtype=np.float64) for b in data['b']]


class DQNAgent(object):
    """DQN with experience replay and a periodically synced target network."""

    def __init__(self, state_dim=24, n_actions=5, hidden=64, gamma=0.95,
                 replay=20000, batch=32, lr=1e-3, seed=7):
        self.n_actions = n_actions
        self.gamma = gamma
        self.batch = batch
        self.lr = lr
        self.replay = deque(maxlen=replay)
        self.q = MLP([state_dim, hidden, hidden, n_actions], seed=seed)
        self.qt = MLP([state_dim, hidden, hidden, n_actions], seed=seed)
        self.eps = 1.0
        self.steps = 0

    def act(self, state, evaluate=False):
        if not evaluate and random.random() < self.eps:
            return random.randrange(self.n_actions)
        q = self.q.predict(np.asarray(state, dtype=np.float64))[0]
        return int(np.argmax(q))

    def remember(self, s, a, r, s2, done):
        self.replay.append((np.asarray(s, dtype=np.float64), a, float(r),
                            np.asarray(s2, dtype=np.float64), bool(done)))

    def decay(self, rate=0.995, floor=0.05):
        self.eps = max(floor, self.eps * rate)

    def train_step(self):
        if len(self.replay) < self.batch:
            return None
        batch = random.sample(self.replay, self.batch)
        S = np.stack([b[0] for b in batch])
        S2 = np.stack([b[3] for b in batch])
        target, _ = self.q.forward(S)
        next_q, _ = self.qt.forward(S2)
        for i, (_, a, r, _, done) in enumerate(batch):
            boot = 0.0 if done else self.gamma * float(np.max(next_q[i]))
            target[i, a] = r + boot
        self.steps += 1
        return self.q.train(S, target, lr=self.lr)

    def update_target(self):
        self.qt.load(self.q.params())

    def save(self, path):
        with open(path, 'w') as f:
            json.dump({'q': self.q.params(), 'eps': self.eps}, f)

    def load(self, path):
        with open(path) as f:
            data = json.load(f)
        self.q.load(data['q'])
        self.qt.load(data['q'])
        self.eps = 0.0
