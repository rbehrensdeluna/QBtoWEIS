# QBtoWEIS

**QBtoWEIS** is an extension of the WEIS (Wind Energy with Integrated Servo-control) framework that integrates the capabilities of QBlade to expand the design and optimization process for floating offshore wind turbines. By adding computationally highly optimized methods for wake aerodynamics and structural modeling, QBtoWEIS creates an even more versatile and powerful multi-fidelity toolchain, offering greater flexibility in co-design and optimization.

## WEIS

[WEIS](https://github.com/WISDEM/WEIS) is a comprehensive framework developed by the National Renewable Energy Laboratory (NREL) that integrates several NREL-developed tools for the design optimization of floating offshore wind turbines. It facilitates aero-servo-hydro-elastic modeling, structural analysis, and control system design, enabling a holistic approach to floating wind turbine development.

## QBlade

[QBlade](https://qblade.org/) is a versatile aero-servo-hydro-elastic simulation tool for wind turbines. It features a highly optimized lifting-line free vortex wake algorithm, which ensures efficient and accurate wake modeling. The underlying structural model implemented in QBlade supports Euler-Bernoulli, Timoshenko and Timoshenko FPM theory, all of which provide computationally efficient structural models. Notably, all three approaches can be associated with similar computational costs, allowing for flexibility in model selection without sacrificing performance.

[QBlade documentation](https://docs.qblade.org/)

## Additional Packages

QBtoWEIS integrates the following packages in addition to the stack of tools already available in WEIS:
* [QBlade](https://qblade.org/) - freely available wind turbine simulation tool

## Additional Packages

* [SONATA](https://github.com/ptrbortolotti/SONATA) - toolbox for Multidisciplinary Rotor Blade Design Environment for Structural Optimization and Aeroelastic Analysis

## **Installation**
The setup process is fully automated. You can install QBtoWEIS with a single command or by using the provided installer scripts for your operating system.

### **Quick Install (Recommended)**

If you want to get started quickly, use the provided installer script for your operating system.

---

> **About Conda**
> QBtoWEIS uses [Conda](https://github.com/conda-forge/miniforge) to manage dependencies and environments cleanly.
> We recommend installing the **Miniforge3** distribution, as it’s lightweight, cross-platform, and uses the community-maintained **conda-forge** channel by default.
>
> If you already have Anaconda or Miniconda installed, you can still use them — just make sure Conda is initialized in your terminal (e.g., run `conda init bash` once).
>
> *Tip:* Using `mamba` instead of `conda` can significantly speed up environment creation. Once Miniforge is installed, you can enable it with:
>
> ```bash
> conda install -n base -c conda-forge mamba
> ```
>
> and then simply replace `conda` with `mamba` in the commands or installer script.

---
Make sure you are in the root directory of the cloned repository before running these commands.

#### **Windows**

Download or clone this repository, then double-click:

```
Install-QBtoWEIS.bat
```

> You can also run it from the command line:
>
> ```cmd
> Install-QBtoWEIS.bat
> ```

#### **Linux / macOS / WSL2**

Download or clone this repository, then run the shell installer:

```bash
chmod +x install_qbtoweis.sh
./install_qbtoweis.sh
```

---

### **Manual Installation (Alternative)**

If you prefer to install everything manually, follow these steps:

```bash
conda config --add channels conda-forge
git clone https://github.com/rbehrensdeluna/QBtoWEIS.git
cd QBtoWEIS
conda env create --name qbweis-env -f environment.yml
conda activate qbweis-env
conda install -y petsc4py mpi4py pyoptsparse # (Mac / Linux only, sometimes Windows users may need to install mpi4py)
pip install -e .
```

Whenever you want to run QBtoWEIS later don't forget to run:

```bash
conda activate qbweis-env
```

---

### **Download QBlade after Installation**

1. Download **QBladeCE (v2.0.9+)**
   → [https://qblade.org/downloads/](https://qblade.org/downloads/)

   Place it in any directory you prefer — we recommend using the same directory as your QBtoWEIS repository.

2. *(Optional but recommended)* Configure the QBlade DLL path.

   Navigate to and open the file in an editor:

   ```
   <directory>/QBtoWEIS/weis/inputs/modeling_schema.yaml
   ```

   Locate the `path2qb_dll` entry.
   Change its default value from:

   ```yaml
   default: None
   ```

   to the full path of your **QBladeEE_2.0.9.dll** (Windows) or **libQBladeEE_2.0.9.so.1.0.0** (Linux/macOS), for example:

   ```yaml
   default: C:\Users\JohnDoe\QBtoWEIS\QBladeCE_2.0.9.4\QBladeCE_2.0.9.4.dll
   ```

3. Save the file.

4. Try a test case in the `qb_examples`:

   ```bash
   cd QBtoWEIS/qb_examples/00_run_test
   python weis_driver_oc3.py
   ```

[Windows installer](install/Install-QBtoWEIS.bat)
[Linux/macOS installer](install/install_qbtoweis.sh)


## Troubleshoot.
If you are having trouble creating the virtual environment try allocating more RAM to the WSL2 (e.g. https://learn.microsoft.com/en-us/answers/questions/1296124/how-to-increase-memory-and-cpu-limits-for-wsl2-win)
