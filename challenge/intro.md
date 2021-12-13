---
version: normal 
---
# Challenge Kick-Off


The challenge aims at making learners develop a first robotic project.
In the end, both the execution of the proposed solution and the source code with documentation will be evaluated.

The main objectives of the project consist of:

- Control a robot in a cluttered environment
- Map a static environment
- Detect all the **Nuka-Cola** cans
- Estimate the position of all the **Nuka-Cola** in the map
- Optimize the exploration strategy

<https://www.myminifactory.com/object/3d-print-fallout-nuka-cola-15616>

![](https://cdn.myminifactory.com/assets/object-assets/579fca2a374fc/images/720X720-7a4418213f3ce580bb21f641c36650bd5eb8cdb3.jpg)

## Create a group

As a first move, you have to constitute a group of 2 developers.

- Log on the proposed virtual tutorial room [on discord](https://discord.gg/dPqtYmvD58)
- Ask a prof to be tagged as a student.
- In resources section find the link to the shared `group file`.
- With my teammate I pick a color and fill the document.

## Generate a working environment

We ask each group to use `git` and share their work through `github`.

Git is a versioning program working in a distributed way.
"Git is software for tracking changes in any set of files, usually used for coordinating work among programmers collaboratively developing source code during software development." [Wikipedia-2021-12-6](https://en.wikipedia.org/wiki/Git).

- Official web site: [git-scm.com](https://git-scm.com/)
- git basics: [in video](https://git-scm.com/videos)

Potentially the developers can share some of the versions there developed (or all) by installing an extra `git` on a shared server. `github` is certainly the most famous web git-solution (but you can install for instance a web application as `gitlab`  on your own server for instance).
For this lecture, you will use `github`

- Create a github account (each of the developers)
- Configure `ssh` access (each of the developers)
   + https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh
- Create a new repository on `github` and invite your teammate (one of the developers)
- Clone locally (each of the developers)
- Make your repository publique and share the url with the prof in your private resource chanel on discord.

### Cloning:

The idea is that your repository matches a meta ROS-package (i.e. a directory composed by other directories each of them matching a ros-package). So create your catkin directory, clone your repository in the src directory and then create as many packages you want.

```bash
mkdir catkin-ws
cd catkin-ws
mkdir src
cd src
git clone github/repo.git grp-color
```

If it is the first clone, initialize a `README.md` at least with a simple tittle (cf. [Markdown syntax](https://fr.wikipedia.org/wiki/Markdown)):

```bash
echo "# grp-`color` repository for the UV-LARM
" > README.md
git add README.md
git commit -am "Initialize README file"
git pull
git push
```

### New package:

Then you can go inside your repository and create a new ros package:

```bash
cd grp-color
catkin_create_pkg my_amasing_pkg std_msgs rospy roscpp
ls
```

A new directory `my_amasing_pkg` appear:

The `git status` command informs you that this package is not followed by `git`. Let correct that.

```bash
git add my_amasing_pkg
git commit -am "Initializing my_amasing_pkg"
```

Strange, nothing changes on my `github` repo.
The git repo on `github` is a different entity than the one on your computer. You have to manually synchronize then when you want to share your work with the command `pull`  and `push`. Try-it.

Now you can `commit`, `pull`, `push` as often as possible (and `add` if you are some new files...).


## Build your package

On a fresh machine, you can clone then build your new ros-package:

```bash
mkdir catkin-ws
cd catkin-ws
mkdir src
git clone github/repo.git src/grp-color
catkin_make
```

To notice that `catkin_make` have to be performed from your catkin root directory (`catkin-ws` in this example). 
To relax this constraint you can add a simple shell script in your working directory...

```bash
cd src/grp-color # go to src/grp-color
touch make.sh    # create a file named make.sh
chmod +x make.sh # make the file executable
```

edit `make.sh` and add the following code:

```bash
#!/usr/bin/env sh

cd ../..
catkin_make

```

Now a `./make.sh` performed from your directory would do the job (you can `add`, `commit`, `pull`, `push`).

<!--
## Agile software development

Agile software development aims at breaking with traditional project management by preferring:

- **Individuals and Interactions** over processes and tools
- **Working Software** over comprehensive documentation
- **Customer Collaboration** over contract negotiation
- **Responding to Change** over following a plan

The main feature of Agile Software Development consists of iterative development by moving forward incrementally
and by delivering operational versions frequently.

More on [WikiPedia](https://en.wikipedia.org/wiki/Agile_software_development).
-->
