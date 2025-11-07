# RSEC - Robustness Study with Embodied Control

## 🚀 Quick Start Commands

### LIBERO Evaluation
```bash
cd LIBERO
python eval.py --args.action-noise-scale 0.5 --args.action-noise-dim action.eef_pos_delta[2]
```

### GR00T Inference Service
```bash
cd Issac-GR00T
python scripts/inference_service.py --server \
    --model_path /mnt/lustre/slurm/users/taywonmin/rsec/model/checkpoint-60000 \
    --embodiment_tag new_embodiment \
    --data_config libero \
    --denoising_steps 4 \
    --port 5555
```

### SLURM Job Submission
```bash
srun --gpus=1 --cpus-per-task=4 --mem=16G --time=03:50:00 --nodelist=node7 --pty bash
```

### Compare before(no noise) vs after(noise)
```bash
cd LIBERO
python analyze/compare_metrics.py \
--before_dir ./analysis/analysis_libero_10_20251107_172053_noise_00000/ \
--after_dir ./analysis/analysis_libero_10_20251107_145142_noise_05000_dim_action.eef_pos_delta[2]/
```

### Result example
See "rsec/(Result_example)comparison_analysis_libero_10_20251107_175236_noise_05010_dim_action.eef_pos_delta_2_.txt"

---

## 📊 Action Noise Configuration

### Attack Dimensions Overview
- **action.eef_pos_delta**: shape (T, 3) → `[x_delta, y_delta, z_delta]`
- **action.eef_rot_delta**: shape (T, 3) → `[roll_delta, pitch_delta, yaw_delta]`
- **action.gripper_close**: shape (T, 2) → `[finger1, finger2]`

### Example Commands

#### End-Effector Position (eef_pos_delta)
```bash
# X-axis only (index 0)
python eval.py --args.action-noise-scale 0.1 --args.action-noise-dim action.eef_pos_delta[0]

# Y-axis only (index 1)
python eval.py --args.action-noise-scale 0.1 --args.action-noise-dim action.eef_pos_delta[1]

# Z-axis only (index 2)
python eval.py --args.action-noise-scale 0.1 --args.action-noise-dim action.eef_pos_delta[2]
```

#### End-Effector Rotation (eef_rot_delta)
```bash
# Roll only (index 0)
python eval.py --args.action-noise-scale 0.05 --args.action-noise-dim "action.eef_rot_delta[0]"

# Pitch only (index 1)
python eval.py --args.action-noise-scale 0.05 --args.action-noise-dim "action.eef_rot_delta[1]"

# Yaw only (index 2)
python eval.py --args.action-noise-scale 0.05 --args.action-noise-dim "action.eef_rot_delta[2]"
```

#### Gripper Control (gripper_close)
```bash
# Finger 1 only (index 0)
python eval.py --args.action-noise-scale 0.1 --args.action-noise-dim "action.gripper_close[0]"

# Finger 2 only (index 1)
python eval.py --args.action-noise-scale 0.1 --args.action-noise-dim "action.gripper_close[1]"
```

---

## 🔧 Environment Setup

### Node 8 Configuration (메모임 안함)
```bash
# Setup robosuite log path
sed -i 's|/tmp/robosuite.log|/home/taywonmin/robosuite.log|' /home/taywonmin/miniconda3/envs/libero/lib/python3.8/site-packages/robosuite/utils/log_utils.py
touch /home/taywonmin/robosuite.log
chmod 600 /home/taywonmin/robosuite.log
```

### Recovery (if needed)
```bash
# Restore original path
sed -i 's|/home/taywonmin/robosuite.log|/tmp/robosuite.log|' /home/taywonmin/miniconda3/envs/libero/lib/python3.8/site-packages/robosuite/utils/log_utils.py
```

### Checkpoint Location
```
/mnt/lustre/slurm/users/taywonmin/rsec/model/gr00tn1.5/checkpoint-60000
```

---

## 🎬 Headless Rendering Setup

### Problem
When running simulation environments in headless environments (Vast.ai, Docker, etc.), OpenGL errors occur:
- `AttributeError: 'NoneType' object has no attribute 'glGetError'`
- `ImportError: Cannot initialize a EGL device display... PLATFORM_DEVICE extension not supported`

### Solution: CPU Rendering with OSMesa

#### Step 1: Install Mesa Libraries
```bash
conda activate libero
conda install -c conda-forge mesalib
```

#### Step 2: Set Environment Variables
```bash
export PYOPENGL_PLATFORM=osmesa
export MUJOCO_GL=osmesa

# Run evaluation
python eval.py
```

---

## 📖 Model Inference Service

### Method 0: Direct Checkpoint Access (via SLURM)
Since this is a SLURM environment, map the checkpoint path directly without downloading.

### Method 1: Download Model from Google Drive

#### Download entire model folder
```bash
# Install gdown
pip install gdown

# Download LIBERO-10 model
gdown --folder https://drive.google.com/drive/folders/1rlbXnm-BtRCHCvghMa7CsrNr0B8-7c0I

# Download LIBERO Object model
gdown --folder https://drive.google.com/drive/u/1/folders/121lEY_nt3PRFFY-qjhPmyhMckWUguMqX

# Move to runs directory
mv libero_object ./runs/libero_object
```

#### Run Inference Service
```bash
python scripts/inference_service.py --server \
    --model_path ./runs/libero_object \
    --embodiment_tag new_embodiment \
    --data_config libero_single_view \
    --denoising_steps 4 \
    --port 5555
```

### Method 2: Mount Google Drive (Temporary Access)

#### Install google-drive-ocamlfuse
```bash
sudo add-apt-repository ppa:alessandro-strada/ppa
sudo apt-get update
sudo apt-get install google-drive-ocamlfuse
```

#### Setup Google OAuth Credentials
1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select an existing one
3. Enable the Google Drive API
4. Go to Credentials → Create Credentials → OAuth Client ID
5. Application Type: Desktop Application
6. Download the client configuration file
7. Run:
```bash
google-drive-ocamlfuse -headless -id YOUR_CLIENT_ID -secret YOUR_CLIENT_SECRET
```

#### Mount Google Drive
```bash
mkdir ./runs/google-drive
google-drive-ocamlfuse ./runs/google-drive
```

#### Run Inference Service
```bash
python scripts/inference_service.py --server \
    --model_path ./runs/google-drive/Isaac-GR00T/runs/libero_10_single_view_seed42 \
    --embodiment_tag new_embodiment \
    --data_config libero_single_view \
    --denoising_steps 4 \
    --port 5555
```

#### Unmount Google Drive
```bash
fusermount -u ./runs/google-drive
```

---

## 📝 LIBERO Evaluation

### Basic Usage
```bash
# First, start the inference server in a separate terminal
python scripts/inference_service.py --server ...

# Then, run the evaluation script
python eval.py
```

### Adding Noise to Actions
To test the robustness of the policy or encourage different behaviors, inject alternating noise into the model's predicted actions using the `action_noise_scale` hyperparameter.

The script adds a pattern of `++ -- ++ --` (positive, positive, negative, negative, ...) noise for each dimension of the action.

**Example:**
```bash
python eval.py --args.action-noise-scale 0.05
```

By default, `action_noise_scale` is `0.0`, meaning no noise is added.

memo
```bash
# Install gdown for Google Drive downloads
pip install gdown
# Download the entire folder (replace FOLDER_ID with the actual ID)
# From the link: https://drive.google.com/drive/u/1/folders/1rlbXnm-BtRCHCvghMa7CsrNr0B8-7c0I
gdown --folder https://drive.google.com/drive/folders/1rlbXnm-BtRCHCvghMa7CsrNr0B8-7c0I #libero_10
gdown --folder https://drive.google.com/drive/u/1/folders/121lEY_nt3PRFFY-qjhPmyhMckWUguMqX 
#libero_object
https://drive.google.com/drive/folders/121lEY_nt3PRFFY-qjhPmyhMckWUguMqX?usp=sharing
```

z -> 2,4  6-> 7