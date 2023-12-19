# Challenge Kick-Off

The challenge aims at making learners develop a first robotic project.
In the end, both the execution of the proposed solution and the source code with documentation will be evaluated.

The main objectives of the project consist of:

- Control a robot in a cluttered environment
- Map a static environment
- Detect all the **Nuka-Cola** bottles
- Estimate the position of all the **Nuka-Cola** in the map
- Optimize the exploration strategy

<!-- <https://www.myminifactory.com/object/3d-print-fallout-nuka-cola-15616> -->

![](https://cdn.myminifactory.com/assets/object-assets/579fca2a374fc/images/720X720-7a4418213f3ce580bb21f641c36650bd5eb8cdb3.jpg)

Challenges are proposed to increase sequentially the complexity of the expected solution,
but first the students have to structure their works environment... 


## Create a group

As a first move, you have to constitute a group of 2 developers.
Record the created group on a shared document: [2022-2023 groups](https://partage.imt.fr/index.php/s/CJgSK6W8izdZKDi)

- Create on items in the enumerate list per groups
- Record the name of the machine you use and the names of each member of the group. 
- The number of the line matches the number identifying a group starting from $1.$. The group gets a unique number it can use as `ROS_DOMAIN_ID`.


## Generate a working environment

We ask each group to use `git` and share their work with professors through [github](https://github.com/) (because it is the reference) (or [gitlab](https://gitlab.com) at the student request because it is open).

Git is a versioning program working in a distributed way.
"Git is software for tracking changes in any set of files, usually used for coordinating work among programmers collaboratively developing source code during software development." [Wikipedia-2021-12-6](https://en.wikipedia.org/wiki/Git).

- Official web site: [git-scm.com](https://git-scm.com/)
- git basics: [in video](https://git-scm.com/videos)

Potentially the developers can share some of the versions there developed (or all) by installing an extra `git` on a shared server. `github` is certainly the most famous web git-solution (but you can install for instance a web application as `gitlab`  on your own server for instance).
To notice that `gitlab` is both an open solution you can deploy on your own server and a web-service (`gitlab.com`) you can use.
For this lecture, you will use `github` or `gitlab.com`, no other solution would be accepted.

- (Each of the developers) Create a github account.
- (Each of the developers) Configure `ssh` access (each of the developers).
   + https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh
- (One of the developers) Create a new repository on `github` and invite your teammate. Choose a name referring the group (for instance `uvlarm-machinename`), the ideas is to propose a repository name clear but different from a group to another.
- (Each of the developers) Clone locally 
- (One of the developers) Invite the professor (or set your repository public) - github account: lozenguez LucFabresse bouraqadi SebAmb
- (One of the developers) Reccord the url in the shared document: [2022-2023 groups](https://partage.imt.fr/index.php/s/zkQbXMsrWdp2RQd)


### Initialize:

Your repository has to match a meta **ROS-package** (i.e. a directory composed by other directories each of them matching a ros-package).
The repository would be cloned aside of the larm packages (`pkg-tbot`).
So clone your repository in the ros workspace `mb6-space` and then create as many packages you want inside.

```console
cd ~/mb6-space
git clone github/uvlarm-machinename.git
```

(One of the developers) Initialize a `README.md` file in Mardown at least with a simple tittle and refering the developers (cf. [Markdown syntax](https://fr.wikipedia.org/wiki/Markdown)):

```console
echo "# grp-`machinename` repository for the UV-LARM" > README.md
git add README.md
```

It is possible then to push on the server your first commit.
One commit refer to one version of your project, you can (must) generate has versions has you want. 
A commit do not have to serve a functional version of your works.
It could be transitive commit. 
And most importantly git is capable to re-generate any of your commits, so do not hesitate to commit continuously...

```console
git commit -am "Initialize README file"
git pull
git push
```

All the other developers can now `pull` the new version (typically, in the Develter computers).... 


### New package:

Then you can go inside your repository and create a new ros package:

```console
cd larm-machinename
ros2 pkg create ... # cf. package tutorial
```

A new directory `my_amasing_pkg` appears:

The `git status` command informs you that this package is not followed by `git`. Let correct that.

```console
git add my_amasing_pkg
git commit -am "Initializing my_amasing_pkg"
```

Strange, nothing changes on my `github` repo.
The git repo on `github` is a different entity than the one on your computer.
You have to manually synchronize then when you want to share your work with the command `pull`  and `push`. Try-it.

Now you can `commit`, `pull`, `push` as often as possible (and `add` if you have some new files...).


## Build your package

On a fresh machine, you can clone then build your new ros-package:

```console
mkdir mysuper_ros_workspace
cd mysuper_ros_workspace
mkdir src
git clone github/repo.git src/grp-color
colcon build
```

To notice that `colconn build` have to be performed from your workspace directory (`mysuper_ros_workspace` in this example, `mb6-space` in this courses aside of _tbot_ and _tsim_ packages).

<!--
To relax this constraint you can add a simple shell script in your working directory...

```consol
cd uvlarm-machinename # go to uvlarm-machinename 
touch build    # create a file named make.sh
chmod +x build # make the file executable
```

edit `build` and add the following code:

```consol
#!/usr/bin/env bash
cd ..
colcon build

```

Now a `./build` performed from the root directory of your repository would do the job (you can `add`, `commit`, `pull`, `push`, `build` from the same location).

< ! - - 

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
