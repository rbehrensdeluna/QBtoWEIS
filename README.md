# **QBtoWEIS**

**QBtoWEIS** is an extension of the WEIS (Wind Energy with Integrated Servo-control) framework that integrates the capabilities of QBlade to expand the design and optimization process for floating offshore wind turbines. By adding computationally highly optimized methods for wake aerodynamics and structural modeling, QBtoWEIS creates an even more versatile and powerful multi-fidelity toolchain, offering greater flexibility in co-design and optimization.

---

## **WEIS**

[WEIS](https://github.com/WISDEM/WEIS) is a comprehensive framework developed by the National Renewable Energy Laboratory (NREL) that integrates several NREL-developed tools for the design optimization of floating offshore wind turbines. It facilitates aero-servo-hydro-elastic modeling, structural analysis, and control system design, enabling a holistic approach to floating wind turbine development.

---

## **QBlade**

[QBlade](https://qblade.org/) is a versatile aero-servo-hydro-elastic simulation tool for wind turbines. It features a highly optimized lifting-line free vortex wake algorithm, which ensures efficient and accurate wake modeling. The underlying structural model implemented in QBlade supports Euler-Bernoulli, Timoshenko and Timoshenko FPM theory, all of which provide computationally efficient structural models. Notably, all three approaches can be associated with similar computational costs, allowing for flexibility in model selection without sacrificing performance.

[QBlade Documentation →](https://docs.qblade.org/)

---

## **Additional Packages**

QBtoWEIS integrates additional open-source packages to extend WEIS capabilities:

* [**QBlade**](https://qblade.org/) — wind turbine aeroelastic simulation tool
* [**SONATA**](https://github.com/ptrbortolotti/SONATA) — toolbox for multidisciplinary rotor blade design, structural optimization, and aeroelastic analysis

---

## **Installation**

The setup process is fully automated once you’ve cloned the repository.

1. Add the `conda-forge` channel:

   ```bash
   conda config --add channels conda-forge
   ```

2. Clone the repository:

   ```bash
   git clone https://github.com/rbehrensdeluna/QBtoWEIS.git
   ```

---

### **About Conda**

QBtoWEIS uses [Conda](https://github.com/conda-forge/miniforge) to manage dependencies and environments cleanly.
We recommend installing the **Miniforge3** distribution — it’s lightweight, cross-platform, and uses the community-maintained **conda-forge** channel by default.

If you already have **Anaconda** or **Miniconda**, you can still use them — just ensure Conda is initialized in your terminal (e.g., run `conda init bash` once).

> *Tip:* Using [`mamba`](https://github.com/mamba-org/mamba) instead of `conda` can significantly speed up environment creation.
> Once Miniforge is installed, you can enable it with:
>
> ```bash
> conda install -n base -c conda-forge mamba
> ```
>
> Then simply replace `conda` with `mamba` in the commands or installer script.

---

### **Quick Install (Recommended)**

If you want to get started quickly, use the provided installer script for your operating system, located in the installation directory.
This will automatically set up the environment and dependencies.

#### **Windows**

Download or clone this repository, then double-click:

```
Install-QBtoWEIS.bat
```

> Or run it from the command line:
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
conda install -y petsc4py mpi4py pyoptsparse   # (Mac/Linux only; Windows users may need manual mpi4py install)
pip install -e .
```

When running QBtoWEIS in the future, activate the environment:

```bash
conda activate qbweis-env
```

---

### **Download QBlade After Installation**

1. Download **QBladeCE (v2.0.9+)**
   → [https://qblade.org/downloads/](https://qblade.org/downloads/)

   Place it anywhere you like — we recommend using the same directory as your `QBtoWEIS` repository.

2. *(Optional but recommended)* Configure the QBlade DLL path:
   Open the following file in an editor:

   ```
   <directory>/QBtoWEIS/weis/inputs/modeling_schema.yaml
   ```

   Locate the `path2qb_dll` entry and update it from:

   ```yaml
   default: None
   ```

   to the full path of your **QBlade** DLL or shared object, for example:

   ```yaml
   default: C:\Users\JohnDoe\QBtoWEIS\QBladeCE_2.0.9.4\QBladeCE_2.0.9.4.dll # Windows
   default: /home/johndoe/QBladeEE_2.0.9.4/libQBladeEE_2.0.9.4.so.1.0.0 # Unix
   ```

3. Save the file.

4. Test your setup with an example case:

   ```bash
   conda activate qbweis-env
   cd QBtoWEIS/qb_examples/00_run_test
   python weis_driver_oc3.py
   ```

---

### **Troubleshooting**

If you experience issues creating the Conda environment (especially under WSL2), try increasing your memory allocation:
🔗 [Microsoft Docs: Increase Memory and CPU Limits for WSL2](https://learn.microsoft.com/en-us/answers/questions/1296124/how-to-increase-memory-and-cpu-limits-for-wsl2-win)



