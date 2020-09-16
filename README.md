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

-**Note :** You'll have to create an intel account to be able to download the libraries

3. Since you have cloned the project in your workspace, your nlopt might not work and might throw a catkin_make error of *linker script*. So to get rid of this issue, move to this path - */home/user_PC_name/your_project_name/src/gen_utilities/libnlopt/install* and delete all files inside this folder. Now, clone the libnlopt library in any folder outside your project folder [libnlopt](https://github.com/stevengj/nlopt.git)

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
By default, this installs the NLopt shared library (libnlopt.so) in /usr/local/lib and the NLopt header file (nlopt.h) in /usr/local/include, as well manual pages and a few other files. Since, we do not want this to happen as our CMakeLists.txt located at */home/your_project_name/src/gen_utilities/gen_utilities* have different and if you want to keep that path, you need to install nlopt library to that path. Follow the script below to install to that path while you are still inside **build** folder created in the above steps:

```shell
cmake -DCMAKE_INSTALL_PREFIX=$/home/user_PC_name/your_project_name/src/gen_utilities/libnlopt/install ..
make
make install
```
Now, you can see *include*, *lib* and *share* folder inside your **install** folder.

**Note :** Change *user_PC_name* to your system name and *your_project_name* to name of your project repository.




### Building

If your project needs some additional steps for the developer to build the
project after some code changes, state them here:

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

## Features

What's all the bells and whistles this project can perform?
* What's the main functionality
* You can also do another thing
* If you get really randy, you can even do this

## Configuration

Here you should write what are all of the configurations a user can enter when
using the project.

#### Argument 1
Type: `String`  
Default: `'default value'`

State what an argument does and how you can use it. If needed, you can provide
an example below.

Example:
```bash
awesome-project "Some other value"  # Prints "You're nailing this readme!"
```

#### Argument 2
Type: `Number|Boolean`  
Default: 100

Copy-paste as many of these as you need.

## Contributing

When you publish something open source, one of the greatest motivations is that
anyone can just jump in and start contributing to your project.

These paragraphs are meant to welcome those kind souls to feel that they are
needed. You should state something like:

"If you'd like to contribute, please fork the repository and use a feature
branch. Pull requests are warmly welcome."

If there's anything else the developer needs to know (e.g. the code style
guide), you should link it here. If there's a lot of things to take into
consideration, it is common to separate this section to its own file called
`CONTRIBUTING.md` (or similar). If so, you should say that it exists here.

## Links

Even though this information can be found inside the project on machine-readable
format like in a .json file, it's good to include a summary of most useful
links to humans using your project. You can include links like:

- Project homepage: https://your.github.com/awesome-project/
- Repository: https://github.com/your/awesome-project/
- Issue tracker: https://github.com/your/awesome-project/issues
  - In case of sensitive bugs like security vulnerabilities, please contact
    my@email.com directly instead of using issue tracker. We value your effort
    to improve the security and privacy of this project!
- Related projects:
  - Your other project: https://github.com/your/other-project/
  - Someone else's project: https://github.com/someones/awesome-project/


## Licensing

One really important part: Give your project a proper license. Here you should
state what the license is and how to find the text version of the license.
Something like:

"The code in this project is licensed under MIT license."
