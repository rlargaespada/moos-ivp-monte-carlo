# moos-ivp-monte-carlo

Implementation of Monte Carlo simulation environment for testing path planning in MOOS-IvP.

## Installation

This code is meant for Ubuntu Linux of MacOS machines running bash, not for Windows. For detailed MOOS-IvP installation instructions, follow the MIT 2.680 Lab 1 steps found [here](https://oceanai.mit.edu/ivpman/pmwiki/pmwiki.php?n=Lab.ClassSetup). Abridged instructions are included below.

**For Mac users only**: install XCode and Homebrew by following sections [6.3](https://oceanai.mit.edu/ivpman/pmwiki/pmwiki.php?n=Lab.ClassSetup#section6.3) and [6.4](https://oceanai.mit.edu/ivpman/pmwiki/pmwiki.php?n=Lab.ClassSetup#section6.4) of the 2.680 Lab 1 notes.

**For all users**: follow [section 8.1](https://oceanai.mit.edu/ivpman/pmwiki/pmwiki.php?n=Lab.ClassSetup#obtaining_ivp) of 2.680 Lab 1 to install MOOS-IvP in your home directory. Run `env_bash.sh` to load all executables into your path, and follow [section 8.2](https://oceanai.mit.edu/ivpman/pmwiki/pmwiki.php?n=Lab.ClassSetup#section8.2) to verify installation was successful.

Run `build.sh` to build executables for this project. `env_bash.sh` will have already added these exeuctables into your path.

From this directory, change to the monte carlo mission directory using `cd missions/monte_carlo`. Verify the proper executables are on your path by running `which MOOSDB` and `which uEvalPlanner`, and running `env_bash.sh` if either `which` command fails. Run `launch.sh` to run the Monte Carlo simulation. Use `launch.sh -h` to see all simulation configuration options.

<!-- TODO: ADD INSTALLATION INSTRUCTIONS FOR EIGEN3, MOSEK, CDD, Bezier-->
<!--EIGEN3: sudo apt-get install libeigen3-dev might work, or download and extract, then run instructions in eigen-3.4.0/INSTALL-->
<!-- CDD: sudo apt-get install libcdd-dev or from [github](https://github.com/cddlib/cddlib)-->