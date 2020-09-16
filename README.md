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


## Installing libraries

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
By default, this installs the NLopt shared library (libnlopt.so) in /usr/local/lib and the NLopt header file (nlopt.h) in /usr/local/include, as well manual pages and a few other files. Since, you do not want this to happen as your default CMakeList located at */home/your_project_name/src/gen_utilities/gen_utilities* have different and if you want to keep that path, you need to install nlopt library to that path. 
![CMakeLists.txt](https://0i6auq.by.files.1drv.com/y4mVdNbCYS-_Vx-RLfCH2Xcd4_xVcsAS8uM9P4faGAYfPyQvRQu3Gke83o4O0tpLLB6XYe7vsmXp9ktKRA75PgzD1C56spakoJ7ySO5bQEmF8RYA0PYngNx8Y2e3soOfuM7WPcgvDuOGmFb2C2j6Tqr_gUfb9XiM1dMeNZmihX48GU5jenbnHknOQzgiWO3GKEfLEHRAHIRArEw7otEbqRjsA?width=900&height=742&cropmode=none)  
Follow the script below to install to that path while you are still inside **build** folder created in the above steps:

```shell
cmake -DCMAKE_INSTALL_PREFIX=$/home/user_PC_name/your_project_name/src/gen_utilities/libnlopt/install ..
make
make install
```
Now, you can see *include*, *lib* and *share* folder inside your **install** folder.
Visit the official [website](https://nlopt.readthedocs.io/en/latest/NLopt_Installation/) to follow the steps mentioned here.

**Note :** Change *user_PC_name* to your system name and *your_project_name* to name of your project repository.


### Major Changes to CMakeLists Files

Before you dive into running *catkin_make*, you still have to make some changes to *CMakeLists.txt* into different projects under src folder as hshown in the image below.
![Image](https://obowrw.by.files.1drv.com/y4mkEgDKY9zO6QhZ7jNrBDpIikG1wvqrYtxe8mqcbeSacHR1oJIAOu-D_5uTOdN47OI03-hO4YdQGlWFp_8_8_f0Idgv3deVpELLISLg2cgdf5wWWgxCCrwIOBkc0c0JtBrpl3BnGGepKCCQ9U3xQq4nx-_gG8cf0jSBxRPJKR9Jqg5AlFtO5VwtP64Im3ma3bxPTgG43bDxd3ZonY6uJjB5w?width=890&height=566&cropmode=none)

Confirm that your **opt** folder looks like this:
![this](https://1o6auq.by.files.1drv.com/y4mN8d52-SVpP0-rjLiIvOT1ked8RTUhbLBSa91N-alEaS3bkKLXPv0YvBgOX5iZNpipsrEU1UNUlYnN5Ca5_O-OKjUIDeZYx50wsTnbTNzhAsgO0mxLg9_a8sio6SQ3IA1pGnDFgLlor9ejWF0kjZtPNLAHWp4lVzg5aE25bcmoQP61T2oC5Ww_COW1iL_BDc28Eg896xvAsyuWdM6pnDRag?width=890&height=567&cropmode=none). 

If your folder only have *compilers_and_libraries_2020*, then you don't have to make changes to *CMakeLists*. Proceed only if your installed intel ibrary looks like the image shown below![intel](https://1y6auq.by.files.1drv.com/y4mKBY8DSRDdOzLkj8EddCFZ15CakEwgV6MBVPpyP8j1NfpypwK384XHXj5Z-gC7QaXp5R_y2fSdVgS7fLFbR7sVtI6wD56UmZdTL02GruviDSt_z9EYSxLotp6ptVP5iMJpkGU8IXEKKyV3XzCWOnVcd7PvQuG2ijwztG4PbinPa-EC1TwPxHKAhKREn4rC-ExKTTyY7Xx5qlKtHH0KNr2fQ?width=890&height=567&cropmode=none) You can find your intel library under *opt* located under Other Locations in Ubuntu 18.04 or higher.
![Other Locations/Computer Image](https://1i6auq.by.files.1drv.com/y4mWLwnraz61mgT3uCexm6_OcoSbygkXJcgkR_-vjo3JfpeeFUrz2JKE9XSKL1OPVTBPwaiTShC3fqhBO9gqBswOncvVCQgJqd06BWmB6ouhMlJ5kAiTA-jLxOKUD92LV8kcCSvc8KiYB0vnDQJBvYAQg9jXNXj1uYuIpuzySAjEwxdDsf-OE4eBNHNQM6CM4s_ClW4QgKxGA13RMwOK-0A9Q?width=890&height=567&cropmode=none) in Ubuntu 18.04 or higher.

**So what to change**
Change the path for *MKL* library in all three *CMakeLists* under src folder shown above.

**Previous line**


```shell
set(PARDISO_DIR /opt/intel/compilers_and_libraries_2020/linux/mkl)
```

![Old Image](https://nhowrw.by.files.1drv.com/y4meQO75m1gQe-6vXNa_VcfztzXDIe0M0y8KX_CcAclryJa5vtvjImjBqmVwtRWrabidxQJhh0i1aY4KzgqtMu1aZoJPqTnozqWJuOvhsvY7vT0wfqqAE466oyghuMYxYi-cY6ZdUQNPMwIfsY00ew6Swucq63MJ3EUmlH_x_Ld7lZR3V32lP9hdRd9M6u3WM5tC8E4l3Fhmd5UzCYktN5v4g?width=900&height=742&cropmode=none)


```shell
set(PARDISO_DIR /opt/intel/compilers_and_libraries_2020.3.279/linux/mkl)
```
![New Image](https://nxowrw.by.files.1drv.com/y4mtXQhoCS88QjA1k_Wdif2TDEd8R2EE19VaHOE9hTL1XJ1ALchT-M5Dsz4G6wOWv5aHaPDr8EGx9hm-PHD85Jj0vQ8uaJLmYOvO-31IZJd0wo4yS-oF4Vhbd6NSfmI9wcG1ctz1otHL6_MW5sWrr_iOjQVnXHxgkw1RmdeygDhq_g_5EPuyZxnjjB96F8F0TTIil23kLUMCHVZWlOAGNxsig?width=900&height=742&cropmode=none)

**Note :** Chnage the path to this {compilers_and_libraries_2020.3.279} or whichever folder that contain under path */inter/compilers_and_libraries_2020.3.xxx/linux/mkl*. Also, change the path for TBB if you think that your TBB is not under the specified path.

### Building and Running

Now you have installed all the necessary files and made necessary changes to all CMakeLists.txt files, you are ready to build your project.


```shell
cd your_project_name
catkin_make
```
And if everything has been done as mentioned above, you should see the following in your terminal

![Result](https://0y6auq.by.files.1drv.com/y4mWUtbwNmDKWphimAAm_v1YC_y9bXK26opEc5O4L897XS815333JUvSIcYaPPOJ2KvLdj7B9rCnHC01n_xuRApa0p65whDcN32FDJoelQONhE22qRVJgaku5Tv0qtsFconwwtmDSNB3u4tfX1svNmXIbRZM2M_EEy-0sk1H5sxkFeS5gq3WHman0q-U1FfWETMuBLztd2CzxTdWxeMrvQNYg?width=734&height=488&cropmode=none)

Now, run the followinf command to see the simulation and various results on Terminal

```shell
cd your_project_name
source devel/setup.bash
roslaunch sheet_model_training simulator_test_clean.launch
```
![Terminal](https://nrowrw.by.files.1drv.com/y4mAQIAFPqHy3bTOLYyT-yrw28_ASRpnsC_03Qasp3CuAAjSqD3IAO3MMPJBTe8sRC_8aEr75IHldO3LRqbhYvvLIFKUFtZ4E1xa0c3EtT7V9MwXgGl81MeVakh3Sc2V1aekkUW6gFQT_hx3wKy8M8RvuWJPm0HooyO8iADyH1svfiW_NVWO1MUTi8lu1mVJTyBTFqI-8LRLuobMvZjCjpuDA?width=644&height=938&cropmode=none)

![Simulator](https://pbowrw.by.files.1drv.com/y4mvuFpqq7jrDwSPrZUwx3B134gAQIkUKXBdcGyDQmmrxh4QM7uG68tPcxLHa1SPyzBAay7O87XhyZprVO9bqLqIdrwEe0PvEh_DeL_cbPC-YzNE62SJKGsAnHTaLQciCiJ7eV_50mfZ3GsK__5Ws4v9MGblwgGbmtnl41gNKt4T9YVnl2xSkova6rVg8FD-rQjTwC5Mf_6mcciOWSO8oNTiA?width=800&height=630&cropmode=none)

HAPPY DEBUGGING & CODING