![Logo of the project](https://viterbischool.usc.edu/wp-content/uploads/2017/08/USC-Viterbi-School-of-Engineering.png)

# Composite Sheet Layup
> Sheet layup simulation micking the real life composite sheet behaviour


## Getting started

To install necessary files for the project, git clone the required source folder into the directory where you want to initialise the project.
**Note :** this is just an example of git clone process. The actual link for your project will be different

```shell
git clone https://github.com/sshaizkhan/composite_ws.git
```


### Downloading Libraries

To get the project up and running, you'll need two make some changes to **libnlopt** library and install few necessary **intel libraries** in your system. The link to install the libraries are below

1.  [Intel MKL Library](https://software.intel.com/content/www/us/en/develop/tools/math-kernel-library/choose-download.html)

2.  [Intel TBB Library](https://software.intel.com/content/www/us/en/develop/tools/math-kernel-library/choose-download.html)

Alternatively try to use this [link](https://registrationcenter.intel.com/en/products/postregistration/?sn=N2R2-4JNSSRNK&Sequence=2811162&encema=Wg/bUFJY2qspv9ef8QA1f/5nHuN+p/CyonGMjLT2nvOwcKXymcD+dWF8XDi6H++TyjHdL2SNzEo=&dnld=t&pass=yes)

**Note :** You'll have to create an intel account to be able to download the libraries

3. Since you have cloned the project in your workspace, your nlopt might not work and might throw a catkin_make error of *linker script*. So to get rid of this issue, move to this path: - */home/user_PC_name/your_project_name/src/gen_utilities/libnlopt/install* and delete all files inside this folder. Now, clone the libnlopt library in any folder outside your project folder [libnlopt](https://github.com/stevengj/nlopt.git)

**Note :** Make sure not to delete *CMakeLists.txt* and *package.xml* files inside your **libnlopt** folder, otherwise you'll face config file error while runnnig *catkin_make* command.


## Installing the above downloaded libraries

Once you have downloaded all the libraries mentioned above, you are set to install them in your system. Follow the instructions below to download each library:

1. Intel **MKL** & **TBB** library: You now have a .tgz files for both MKL and TBB in your downloaded folder. Unzip the folders and open the terminal in the unziped folder to run the install script. To install the folder in your *opt* folder, add **sudo** to the installation script.
```shell
sudo ./install.sh
```
or 

```shell
sudo ./install_GUI.sh
```

In the folder, you'll find two installtion files, **install.sh** and **install_GUI.sh**. As the names suggests, the former will run in terminal and the latter one will open a nice GUI for installation. If you'll follow the steps, your system should now have successfully installed 

2. nlopt Library: After you have cloned the nlopt library in your system, follow the instructions below to successfully install the nlopt library:

Run the below commands in terminal inside your cloned nlopt librart

```shell
mkdir build
cd build
```
By default, this installs the NLopt shared library (libnlopt.so) in /usr/local/lib and the NLopt header file (nlopt.h) in /usr/local/include, as well manual pages and a few other files. Since, we do not want this to happen as our [CMakeLists.txt]!(https://ibb.co/mtmrj0H)  located at */home/your_project_name/src/gen_utilities/gen_utilities* have different and if you want to keep that path, you need to install nlopt library to that path. Follow the script below to install to that path while you are still inside **build** folder created in the above steps:

```shell
cmake -DCMAKE_INSTALL_PREFIX=$/home/user_PC_name/your_project_name/src/gen_utilities/libnlopt/install ..
make
make install
```
Now, you can see *include*, *lib* and *share* folder inside your **install** folder.

**Note :** Change *user_PC_name* to your system name and *your_project_name* to name of your project repository.


### More Editing before building

Before you dive into running *catkin_make*, you still have to make some changes to *CMakeLists.txt* into different projects under src folder. View this ![image](https://ibb.co/tZ7RMzd) to know which folder contain the CMakeLists to make changes.

```shell
./configure
make
make install
```

Here again you should state what actually happens when the code above gets
executed.

### Deploying / Publishing

In case there's some step you have to take that publishes this project to a
server, this is the right time to state it.

```shell
packagemanager deploy awesome-project -s server.com -u username -p password
```

And again you'd need to tell what the previous code actually does.

