---
title: Isaac Lab Eureka
sidebar_label: Overview
---

# Isaac Lab Eureka :rocket:

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.5.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.1.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Windows platform](https://img.shields.io/badge/platform-windows--64-orange.svg)](https://www.microsoft.com/en-us/)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](https://opensource.org/license/mit)

Isaac Lab Eureka is an implementation of  
**“[Eureka: Human-Level Reward Design via Coding Large-Language Models](https://github.com/eureka-research/Eureka)”**  
for NVIDIA Omniverse **Isaac Lab**.  
The library prompts an LLM to *discover* and *tune* reward functions automatically for your own RL task.

---

## What You’ll Find Here

| Section | Description |
|---------|-------------|
| **Installation** | Set-up instructions and API-key requirements. |
| **Running** | How to launch training / inference with OpenAI *and* Azure OpenAI back-ends. |
| **Directory Structure** | Tour of the repo so you know where logs, configs, and source live. |
| **Limitations** | Current feature gaps and OS-specific caveats. |
| **Code Formatting** | Pre-commit hooks to keep the codebase tidy. |

Jump to **Installation** to get started!
```

────────────────────────────────────────
2) New dedicated pages
────────────────────────────────────────
```markdown:docs/installation.md
---
title: Installation
---

# Installation

1. **API key**  
   • OpenAI: create a key at <https://platform.openai.com/api-keys>  
   • Azure OpenAI: follow the quick-start at <https://learn.microsoft.com/azure/ai-services/openai/quickstart>

2. **Isaac Lab**  
   Follow the official guide: <https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html>

3. **Isaac Lab Eureka** — install *editable* from the repo root:

```bash
python -m pip install -e source/isaaclab_eureka
```
```

```markdown:docs/running.md
---
title: Running Isaac Lab Eureka
sidebar_label: Running
---

# Running Isaac Lab Eureka

The API key must be exported as an environment variable.

## OpenAI back-end

<details open>
<summary>Linux</summary>

```bash
OPENAI_API_KEY=your_key \
python scripts/train.py --task=Isaac-Cartpole-Direct-v0 --max_training_iterations=100 --rl_library="rl_games"
```

</details>

<details>
<summary>Windows (PowerShell)</summary>

```bash
$env:OPENAI_API_KEY="your_key"
python scripts\train.py --task=Isaac-Cartpole-Direct-v0 --max_training_iterations=100 --rl_library="rl_games"
```

</details>

## Azure OpenAI back-end

<details open>
<summary>Linux</summary>

```bash
AZURE_OPENAI_API_KEY=your_key \
AZURE_OPENAI_ENDPOINT=azure_endpoint_url \
python scripts/train.py --task=Isaac-Cartpole-Direct-v0 --max_training_iterations=100 --rl_library="rl_games"
```

</details>

### Inference on a trained policy

```bash
python scripts/play.py \
  --task=Isaac-Cartpole-Direct-v0 \
  --checkpoint=/path/to/desired/checkpoint.pth \
  --num_envs=20 \
  --rl_library="rl_games"
```
```

```markdown:docs/directory_structure.md
---
title: Directory Structure
sidebar_label: Directory Structure
---

# Repository Layout

```
IsaacLabEureka/
├── LICENCE
├── README.md
├── logs/                  ← Eureka & RL-runs
│   ├── eureka/            ← raw LLM output + metrics
│   └── rl_runs/           ← policy checkpoints
├── scripts/               ← train.py • play.py • helper tools
├── source/
│   └── isaaclab_eureka/   ← pip-installable package
│       ├── config/
│       ├── docs/
│       └── ...
└── test.py
```
```

```markdown:docs/limitations.md
---
title: Limitations
---

# Current Limitations

* Only **direct-workflow** tasks (`DirectRLEnv`) are supported.  
  See `source/isaaclab_eureka/isaaclab_eureka/config/tasks.py` for examples.
* RL libraries: **RSL-RL** and **RL-Games** only.
* Windows: `num_parallel_runs > 1` is **not** supported (multiprocessing).
* Single-GPU: parallel runs will compete for CPU & RAM.
* “Best” policy is picked via each task’s `success_metric` — make it good!
* LLM-generated reward code may contain syntax / logic bugs; such iterations are skipped.
```

```markdown:docs/code_formatting.md
---
title: Code Formatting
sidebar_label: Code Formatting
---

# Code Formatting

We use **pre-commit** to auto-format the code base.

```bash
pip install pre-commit
pre-commit run --all-files
```
```

────────────────────────────────────────
3) Update the sidebar
────────────────────────────────────────
```javascript:sidebars.js
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
module.exports = {
  tutorialSidebar: [
    'index',                 // docs/index.md
    'installation',          // docs/installation.md
    'running',               // docs/running.md
    'directory_structure',   // docs/directory_structure.md
    'limitations',           // docs/limitations.md
    'code_formatting',       // docs/code_formatting.md
  ],
};
