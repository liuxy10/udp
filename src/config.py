import json
import numpy as np


class Config:
    def __init__(self, cfg_path):
        import json
        with open(cfg_path, 'r') as f:
            cfg = json.load(f)
        self.targets = cfg.get("state_targets", {})
        self.target_names = list(self.targets.keys())
        self.state_limit = list(cfg.get("state_limit", {}).values())
        self.param_limit = list(cfg.get("param_limit", {}).values())
        self.param_names = list(cfg.get("param_limit", {}).keys())
        self.param_active = cfg.get("param_active", [True]*len(self.param_names))
        self.param_default = cfg.get("param_default", {})
        assert len(self.targets) == len(self.state_limit), "State targets and limits must have the same length"
        self.state_dim = len(self.targets)
        self.action_dim = sum(self.param_active.values())

        self.target_values = np.array([self.targets[name] for name in self.target_names])
        self.normalized_target_values = np.array([
            (self.targets[name] - self.state_limit[i][0]) / (self.state_limit[i][1] - self.state_limit[i][0])
            for i, name in enumerate(self.target_names)
        ])
        self._get_lspi_cfg(cfg)
        


    def _get_lspi_cfg(self, cfg):
        if "lspi_cfg" in cfg:
            lspi_cfg = cfg["lspi_cfg"]
            self.lspi_cfg = lambda: None
            self.lspi_cfg.gamma = lspi_cfg.get("gamma", 0.95)
            self.lspi_cfg.memory_size = lspi_cfg.get("memory_size", 300)
            self.lspi_cfg.memory_type = lspi_cfg.get("memory_type", "sample")
            self.lspi_cfg.eval_type = lspi_cfg.get("eval_type", "batch")

        else:
            raise ValueError("LSPI configuration 'lspi_cfg' not found in the config file.")
        return self.lspi_cfg
            